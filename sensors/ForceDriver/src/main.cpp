#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "HX711.h"

// Placa: Seeed Studio XIAO ESP32S3 + HX711 (célula de carga).
// Antena externa U.FL OBRIGATÓRIA — sem ela o WiFi não conecta.

// Modo de teste: compile com -DSERIAL_TEST=1 (env serial_test). Pula
// WiFi/OTA/UDP e imprime counts crus + v_sensor no monitor USB (115200).
#ifndef SERIAL_TEST
#define SERIAL_TEST 0
#endif

// ── WiFi — rede do laboratório ──────────────────────────────────────────
// Ao trocar de ambiente, mudar ssid/password e os 4 IPs abaixo JUNTO com
// LOAD_CELL_ESP_IP (constants.py) e upload_port do [env:ota] (platformio.ini).
// Em casa: "Martins 6" / "17031998", rede 192.168.6.0/24, ESP .6.105.
const char* ssid     = "Ender 3 V2 - coleta";
const char* password = "Biolabeb0608";

static const IPAddress LOCAL_IP (192, 168, 5, 105);
static const IPAddress GATEWAY  (192, 168, 5,   1);
static const IPAddress SUBNET   (255, 255, 255,  0);
static const IPAddress BCAST_IP (192, 168, 5, 255);
#define UDP_PORT 8080

// Auto-descoberta: receptores mandam "FRCV" para DISCOVERY_PORT a cada ~2 s;
// cada um ocupa/renova um slot em g_subs e recebe a telemetria por UNICAST
// (broadcast em WiFi perde ~30%). Sem assinante fresco, cai no broadcast.
#define DISCOVERY_PORT     8090
#define DISCOVERY_MAGIC    "FRCV"
#define HELLO_TIMEOUT_MS   10000
#define MAX_SUBSCRIBERS    4

WiFiUDP udp;
WiFiUDP udpRx;                       // escuta do hello

struct Subscriber {
    IPAddress ip;
    uint32_t  last_hello_ms;
    bool      used;
};
static Subscriber g_subs[MAX_SUBSCRIBERS];

// ── HX711 ───────────────────────────────────────────────────────────────
// DT → D1 (GPIO2), SCK → D3 (GPIO4). Não mover o DOUT para GPIO0/3/45/46:
// são strapping pins do ESP32-S3 (GPIO3 = D2 do XIAO!).
#define HX_DOUT_PIN  2
#define HX_SCK_PIN   4
#define HX_GAIN      128     // canal A

static HX711 hx;

// v_sensor = counts·AVDD/2²⁴ = tensão da ponte já ×PGA(128). HX_VREF é o
// AVDD do módulo (3V3 do XIAO); erro aqui é só escala global, absorvida
// pela calibração da GUI. MANTER SINCRONIZADO com LC_FW_VOLTAGE_SCALE /
// LC_FW_VOLTAGE_OFFSET do constants.py.
const float HX_VREF     = 3.3f;
const float COUNTS_TO_V = HX_VREF / 16777216.0f;

// ── Auto-zero de boot ───────────────────────────────────────────────────
// A célula é bidirecional e o zero elétrico da ponte cai em qualquer lugar
// (inclusive negativo). O firmware trava o repouso como offset ao LIGAR:
// v_sensor transmitido = v_bruto − offset, então a GUI recebe ~0 V em
// repouso e massa aplicada vira tensão > 0. REQUISITO: ligar/re-zerar com
// a célula EM REPOUSO — carga presente no boot é engolida pelo offset.
// Comando 'Z' na USB refaz o zero sob demanda (botão na GUI).
// A coleta exige janela estável (faixa ≤ ZERO_STABLE_V) e recomeça sozinha
// enquanto o sinal deriva; nenhuma amostra é transmitida sem zero travado.
#define ZERO_SAMPLES   40       // ~0,5 s @ 80 Hz
#define ZERO_STABLE_V  0.002f   // faixa máx aceita durante a coleta (V)
static float g_v_offset = 0.0f;
static bool  g_zeroed   = false;
static float g_zero_acc = 0.0f, g_zero_min = 0.0f, g_zero_max = 0.0f;
static int   g_zero_cnt = 0;

static void zero_restart()
{
    g_zeroed   = false;
    g_zero_cnt = 0;
}

// Amostra na rede: little-endian '<IIf' (12 B), 1 por datagrama — formato
// espelhado no force_receiver e no classificador externo. A taxa é ditada
// pelo pino RATE do HX711 (GND = 10 Hz, VDD = 80 Hz); o receptor mede a
// taxa real pelo t_us.
struct __attribute__((packed)) Sample {
    uint32_t seq;
    uint32_t t_us;      // micros() na leitura (relógio de sincronização)
    float    v_sensor;
};

static Sample   sample_out;
static uint32_t tx_seq = 0;

const uint32_t WIFI_RETRY_MS = 3000;
static uint32_t last_wifi_retry_ms = 0;

// Conexão inicial (bloqueante, só no boot); em operação usa wifi_kick().
static void wifi_connect()
{
    WiFi.mode(WIFI_STA);
    WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    // Modem-sleep atrasa/derruba UDP (perda de 5–37% medida) — desligado.
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(500);
    }
    last_wifi_retry_ms = millis();
}

// Atende os hellos pendentes: renova o slot do remetente ou ocupa um
// livre/expirado (lotado: rouba o mais antigo). Não-bloqueante.
static void discovery_poll()
{
    int psize;
    while ((psize = udpRx.parsePacket()) > 0) {
        char buf[8] = {0};
        int n = udpRx.read((uint8_t*)buf, sizeof(buf) - 1);
        if (n < (int)sizeof(DISCOVERY_MAGIC) - 1 ||
            strncmp(buf, DISCOVERY_MAGIC, sizeof(DISCOVERY_MAGIC) - 1) != 0)
            continue;

        IPAddress src = udpRx.remoteIP();
        uint32_t  now = millis();
        int slot = -1, oldest = 0;
        for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
            if (g_subs[i].used && g_subs[i].ip == src) { slot = i; break; }
            if (!g_subs[i].used ||
                now - g_subs[i].last_hello_ms >= HELLO_TIMEOUT_MS) {
                if (slot < 0) slot = i;
            }
            if (now - g_subs[i].last_hello_ms >
                now - g_subs[oldest].last_hello_ms) oldest = i;
        }
        if (slot < 0) slot = oldest;
        g_subs[slot].ip            = src;
        g_subs[slot].last_hello_ms = now;
        g_subs[slot].used          = true;
    }
}

// Reconexão não-bloqueante, throttled a WIFI_RETRY_MS.
static void wifi_kick()
{
    uint32_t now = millis();
    if (now - last_wifi_retry_ms < WIFI_RETRY_MS) return;
    last_wifi_retry_ms = now;
    WiFi.begin(ssid, password);
}

// ── OTA ─────────────────────────────────────────────────────────────────
// 1ª gravação por USB; depois `pio run -e ota -t upload`.
#define OTA_HOSTNAME  "forcedriver"
#define OTA_PASSWORD  "Biolabeb0608"

static void ota_setup()
{
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
}

void setup()
{
    hx.begin(HX_DOUT_PIN, HX_SCK_PIN, HX_GAIN);
    // Pull-up no DOUT: sem HX711 o pino flutua em LOW (= "amostra pronta")
    // e o loop despejaria zeros falsos; com pull-up, ausência = silêncio.
    pinMode(HX_DOUT_PIN, INPUT_PULLUP);
#if SERIAL_TEST
    // LED do XIAO (GPIO21, aceso em LOW): alterna a cada amostra lida;
    // piscando a 1 Hz = parado no heartbeat, sem amostra.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.begin(115200);
    delay(2000);               // tempo do monitor USB CDC enumerar/abrir
    Serial.println("[serial_test] HX711 DT=GPIO2(D1) SCK=GPIO4(D3), rádio OFF");
    Serial.println("[serial_test] esperando amostras (DOUT precisa pulsar)...");
#else
    // Serial SEMPRE ativa em paralelo ao UDP: "WiFi conectado" não prova
    // entrega (sub-rede errada / firewall no PC); a GUI deduplica (ignora a
    // serial com UDP fresco). TxTimeout 0: sem host lendo, descarta sem
    // bloquear a amostragem.
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    wifi_connect();
    ota_setup();
    udpRx.begin(DISCOVERY_PORT);
#endif
}

// Unicast a cada assinante fresco; broadcast se não houver nenhum.
static void send_sample()
{
    uint32_t now = millis();
    bool sent = false;
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (!g_subs[i].used ||
            now - g_subs[i].last_hello_ms >= HELLO_TIMEOUT_MS) continue;
        udp.beginPacket(g_subs[i].ip, UDP_PORT);
        udp.write(reinterpret_cast<const uint8_t*>(&sample_out),
                  sizeof(Sample));
        udp.endPacket();
        sent = true;
    }
    if (!sent) {
        udp.beginPacket(BCAST_IP, UDP_PORT);
        udp.write(reinterpret_cast<const uint8_t*>(&sample_out),
                  sizeof(Sample));
        udp.endPacket();
    }
}

void loop()
{
#if !SERIAL_TEST
    const bool wifi_up = (WiFi.status() == WL_CONNECTED);

    // Heartbeat de diagnóstico (0.5 Hz): estado do WiFi + nº de assinantes
    // com hello fresco (prova de que um receptor nos alcança) + contador de
    // amostras. Linhas '#' são ignoradas pelo parser da GUI.
    static uint32_t last_hb_ms = 0;
    uint32_t hb_ms = millis();
    if (hb_ms - last_hb_ms >= 2000) {
        last_hb_ms = hb_ms;
        int subs = 0;
        for (int i = 0; i < MAX_SUBSCRIBERS; i++)
            if (g_subs[i].used &&
                hb_ms - g_subs[i].last_hello_ms < HELLO_TIMEOUT_MS) subs++;
        Serial.printf("# wifi=%s status=%d subs=%d amostras=%lu "
                      "offset=%.6f zeroed=%d\n",
                      wifi_up ? "up" : "down",
                      (int)WiFi.status(), subs, (unsigned long)tx_seq,
                      g_v_offset, (int)g_zeroed);
    }

    if (!wifi_up) {
        wifi_kick();
    } else {
        // OTA + auto-descoberta a ~20 Hz, fora do caminho quente.
        static uint32_t last_house_ms = 0;
        uint32_t house_ms = millis();
        if (house_ms - last_house_ms >= 50) {
            last_house_ms = house_ms;
            ArduinoOTA.handle();
            discovery_poll();
        }
    }
#else
    // Heartbeat 1 Hz enquanto não chega amostra, com o nível do DOUT:
    // preso em HIGH = HX711 ausente/DT errado; preso em LOW = SCK errado
    // ou DT em curto.
    static uint32_t last_beat_ms = 0;
    static uint32_t last_sample_seq = 0;
    uint32_t beat_ms = millis();
    if (beat_ms - last_beat_ms >= 1000) {
        last_beat_ms = beat_ms;
        if (tx_seq == last_sample_seq) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            Serial.printf("[serial_test] sem amostra ha 1 s; DOUT=%s\n",
                          digitalRead(HX_DOUT_PIN) ? "HIGH (HX711 ausente/DT errado?)"
                                                   : "LOW (SCK errado/DT em curto?)");
        }
        last_sample_seq = tx_seq;
    }
#endif

    // Handshake real do HX711: só lê com DOUT baixo APÓS tê-lo visto alto
    // desde a última leitura — um DOUT preso em LOW (curto/fiação errada)
    // trava aqui em vez de virar stream de zeros. read() só relojoa os
    // 24 bits (~60 µs); nunca espera conversão. No SERIAL_TEST o handshake
    // é dispensado de propósito (queremos ler continuamente p/ diagnóstico).
#if SERIAL_TEST
    if (!hx.is_ready()) return;
#else
    static bool dout_seen_high = false;
    if (!hx.is_ready()) { dout_seen_high = true; return; }
    if (!dout_seen_high) return;
    dout_seen_high = false;
#endif
    long counts = hx.read();
    uint32_t now_us = micros();

    float v_raw = (float)counts * COUNTS_TO_V;

#if !SERIAL_TEST
    // Comando de re-zero pela USB ('Z'): a GUI (ou o monitor serial) pede um
    // novo zero com a célula em repouso. Lido aqui, no caminho da amostra,
    // p/ não depender do housekeeping (que só roda com WiFi up).
    while (Serial.available() > 0) {
        if (Serial.read() == 'Z') zero_restart();
    }

    if (!g_zeroed) {
        if (g_zero_cnt == 0) {
            g_zero_acc = 0.0f;
            g_zero_min = g_zero_max = v_raw;
        }
        g_zero_acc += v_raw;
        if (v_raw < g_zero_min) g_zero_min = v_raw;
        if (v_raw > g_zero_max) g_zero_max = v_raw;
        g_zero_cnt++;
        if (g_zero_max - g_zero_min > ZERO_STABLE_V) {
            g_zero_cnt = 0;            // sinal derivando — recomeça a coleta
        } else if (g_zero_cnt >= ZERO_SAMPLES) {
            g_v_offset = g_zero_acc / (float)g_zero_cnt;
            g_zeroed   = true;
            Serial.printf("# zero travado: offset=%.6f V (faixa %.3f mV)\n",
                          g_v_offset, (g_zero_max - g_zero_min) * 1e3f);
        }
        return;   // sem zero travado, nenhuma amostra é transmitida
    }
#endif

    float v_sensor = v_raw - g_v_offset;

    sample_out.seq      = tx_seq++;
    sample_out.t_us     = now_us;
    sample_out.v_sensor = v_sensor;
#if SERIAL_TEST
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // counts crus diagnosticam a fiação: -1 = DT com mau contato/sem GND
    // comum; 0 = SCK não chega; ±8388607 = saturado (A+/A− trocados ou
    // ponte solta); variando com o toque = OK. Prints a ~20 linhas/s.
    static uint32_t last_print_ms   = 0;
    static uint32_t win_samples     = 0;
    win_samples++;
    uint32_t print_ms = millis();
    uint32_t dt_ms = print_ms - last_print_ms;
    if (dt_ms >= 50) {
        Serial.printf("seq=%lu  counts=%ld  v_sensor=%.6f V  (~%.0f amostras/s)\n",
                      (unsigned long)sample_out.seq, counts, v_sensor,
                      win_samples * 1000.0f / (float)dt_ms);
        last_print_ms = print_ms;
        win_samples   = 0;
    }
#else
    if (wifi_up) {
        send_sample();
    }
    // Serial sempre: a mesma amostra como texto, formato espelhado em
    // touch_pack/lc_serial.py (F,<seq>,<t_us>,<v_sensor>).
    Serial.printf("F,%lu,%lu,%.7f\n",
                  (unsigned long)sample_out.seq,
                  (unsigned long)sample_out.t_us,
                  (double)sample_out.v_sensor);
#endif
}
