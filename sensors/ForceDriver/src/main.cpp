#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ======================================================
// WIFI — rede local do laboratório
// ======================================================
const char* ssid     = "Ender 3 V2 - coleta";
const char* password = "Biolabeb0608";

static const IPAddress LOCAL_IP (192, 168, 5, 105);
static const IPAddress GATEWAY  (192, 168, 5,   1);
static const IPAddress SUBNET   (255, 255, 255,  0);

static const IPAddress BCAST_IP (192, 168, 5, 255);
#define UDP_PORT 8080

// Auto-descoberta: o force_receiver manda um "hello" (tag 'FRCV') para
// DISCOVERY_PORT; gravamos o IP do remetente e passamos a enviar a telemetria
// por UNICAST de volta — o WiFi reconhece/retransmite unicast, então a perda
// (que era ~30% no broadcast) cai para perto de zero. Se nunca recebermos um
// hello (ou ele ficar obsoleto > HELLO_TIMEOUT_MS), caímos de volta no
// broadcast, então o sistema funciona mesmo antes da descoberta.
//
// MÚLTIPLOS ASSINANTES (13/07): antes havia UM destino (o último hello
// vencia), então dois PCs ouvindo (touch_pack + classificador no Windows)
// ficavam alternando o stream a cada hello (~2 s de dados p/ cada um).
// Agora cada hello ocupa/renova um slot em g_subs e o lote é enviado por
// unicast a TODOS os assinantes frescos (2 PCs → ~200 pkt/s, tranquilo p/ o
// link). Sem nenhum assinante fresco, cai no broadcast como antes.
#define DISCOVERY_PORT     8090
#define DISCOVERY_MAGIC    "FRCV"
#define HELLO_TIMEOUT_MS   10000
#define MAX_SUBSCRIBERS    4

WiFiUDP udp;
WiFiUDP udpRx;                       // socket de escuta do hello

struct Subscriber {
    IPAddress ip;
    uint32_t  last_hello_ms;
    bool      used;
};
static Subscriber g_subs[MAX_SUBSCRIBERS];

// ======================================================
// ADC — célula de carga via amplificador no GPIO 34
// ======================================================
#define ADC_PIN       34

// Oversampling CONTÍNUO com analogReadMilliVolts() (mV calibrados pela eFuse).
//
// NOTA: tentamos analogRead() cru, mas nesta placa/core ele retornava 0 (pino
// grudava no piso) — voltamos para analogReadMilliVolts(), que lê certo e
// ainda lineariza o ADC pela curva de fábrica. Ele devolve mV INTEIROS (passo
// de 1 mV), MAS acumulamos centenas de leituras por tick em ponto flutuante:
// como o ruído do ADC é > 1 LSB, ele funciona como dither e a média recupera
// frações de mV (resolução efetiva ~0.1-0.2 mV no pino → bem abaixo do passo
// de 1 mV). É a forma mais barata e confiável de aumentar a resolução.
static uint64_t adc_sum_mv = 0;   // soma de mV calibrados
static uint32_t adc_count  = 0;

// DIAGNÓSTICO: se 1, envia a tensão CRUA do pino (V, média do oversampling,
// SEM ganho/offset/mediana/EMA). Nesse modo o repouso aparece em ~0.142 V
// (= V_OFFSET / V_GAIN, o offset do amp visto DEPOIS do divisor), que é o
// "offset alto" observado na GUI. Em PRODUÇÃO (0) o firmware subtrai V_OFFSET
// e filtra, então v_sensor sai ~0 sem carga e usa a escala inteira (5 N em
// 1000 N ≈ 50 mV no domínio do amp, muito acima do passo do ADC). Só volte a
// 1 para diagnosticar o hardware.
#define DIAG_RAW  0

// FILTRAGEM MOVIDA PARA O PC (force_receiver_node). A 1 kHz o filtro pesado
// (mediana + EMA dupla) é definido em AMOSTRAS, então teria de ser reajustado a
// cada mudança de taxa e exigiria reflashar o ESP para qualquer tweak. Aqui
// fica só o filtro LEVE — a média do oversampling do ADC dentro de cada janela
// de 1 ms (ver loop()) — que é praticamente de graça e dá o dither sub-mV. O
// force_receiver aplica mediana + EMA em software, onde é trivial reajustar.
// ======================================================
// RECONSTRUÇÃO DA SAÍDA DO AMPLIFICADOR (V_GAIN) — CALIBRADO POR MEDIÇÃO
// ======================================================
// O firmware lê a tensão no pino do ADC (v_adc, DEPOIS do divisor) e reconstrói
// a saída do amplificador: v_sensor = v_adc * V_GAIN. É v_sensor que trafega na
// rede e aparece como "LC Voltage" na GUI — logo v_sensor DEVE bater com o que
// você mede com o multímetro na saída do amp.
//
// >>> O divisor É modelado por R1/R2, mas AFERIDO pelo ADC — não pelo nominal. <<<
// V_GAIN = (R1+R2)/R2 reconstrói a saída do amp a partir do que o ADC lê.
//
// ARMADILHA (é o que gerou os ganhos errados anteriores): NÃO meça o pino do ADC
// com multímetro para achar o ganho. O ADC do ESP32 tem impedância de entrada
// FINITA, que entra em paralelo com R2 e carrega o divisor — a tensão que o
// multímetro (alta-Z) lê no pino NÃO é a que o ADC converte. Ex.: multímetro no
// pino deu 0.178 V, mas o ADC na prática reporta ~0.257 V. Calibrar pelo pino
// (0.178) deu V_GAIN=1.9438 e a GUI foi para ~0.5 V; o certo é calibrar pelo
// próprio ADC.
//
// Como reaferir (SÓ a razão R1/R2 importa — não dá p/ recuperar os dois físicos,
// e o R1 aqui é EFETIVO, já embute o carregamento do ADC, ≠ resistor da placa):
//   1) multímetro na SAÍDA do amp com um peso parado → V_amp   (ex.: 0.345 V)
//   2) leia v_adc que o ADC reporta = (Raw Voltage da GUI) / (V_GAIN atual)
//   3) V_GAIN_alvo = V_amp / v_adc  →  ajuste R1 até (R1+R2)/R2 = V_GAIN_alvo
// Aferido 2026-07-10, 500 g: V_amp=0.345 V, v_adc=0.5/1.9438=0.2572 V →
// V_GAIN=1.341. Com R2=98600 fixo, R1=33650 dá (33650+98600)/98600=1.3413 e a
// GUI mostra 0.2572·1.3413 = 0.345 V, batendo com o amp.
// Depois de mudar isto é OBRIGATÓRIO refazer a calibração na GUI (slope/intercept
// dependem desta escala). Manter constants.py (LC_FW_VOLTAGE_SCALE) sincronizado.
//
// SEM_DIVISOR: a saída do amp em 0..2520 g é 0.076..2.034 V, que cabe INTEIRA nos
// 0..3.3 V do ADC — o divisor é desnecessário e só joga resolução fora e injeta
// ruído (~80 mV pp no pino). O IDEAL é remover o divisor (amp direto no GPIO34) e
// pôr SEM_DIVISOR=1 → V_GAIN=1.0 e v_sensor = a própria saída do amp.
#define SEM_DIVISOR 0
#if SEM_DIVISOR
const float V_GAIN = 1.0f;             // amp direto no GPIO34 (sem divisor)
#else
const float R1 = 33650.0f;             // EFETIVO (aferido p/ V_GAIN=1.341), ≠ nominal
const float R2 = 98600.0f;             // resistor físico (perna baixa do divisor)
const float V_GAIN = (R1 + R2) / R2;   // = 1.3413 (MEÇA V_amp/v_adc p/ reaferir)
#endif

// Offset DC do amplificador em repouso (sem carga). Com a célula de carga de
// 5 kg (saída nominal 1.0 mV/V) a saída repousava em ~0.4544 V numa montagem
// anterior. Subtraímos para que a tensão enviada seja ~0
// sem força. Medido em produção (DIAG_RAW=0) sobrava +0.00446 V de resíduo no
// repouso → somado (0.4544 + 0.00446 = 0.45886). Re-aferido em 2026-06-21 com a
// célula em repouso (porta 8080): ainda sobravam +0.001416 V → somado de novo
// (0.45886 + 0.001416 = 0.460276), agora o repouso sai em 0. Se mudar de
// célula/amp ou houver deriva térmica, reajuste (a GUI ainda faz tare por cima).
// Aferido 2026-07-10 com a célula de 5 kg EM REPOUSO (sem carga): o v_sensor
// repousava em ~0.190461 V. Subtraímos esse valor para o repouso sair em ~0.
// ATENÇÃO: este offset DERIVA (já foi visto pular 0.077 → 0.190 entre sessões,
// por temperatura/pré-carga da montagem) — se o repouso não sair em 0 depois de
// regravar, reaferir aqui, OU simplesmente usar a tare da GUI, que zera por cima
// sem reflashar. Manter constants.py (LC_FW_VOLTAGE_OFFSET) sincronizado.
const float V_OFFSET = 0.190461f;

// ======================================================
// TEMPORIZAÇÃO — 1 kHz não-bloqueante + ENVIO EM LOTE
// ======================================================
// Amostra a cada 1 ms (1 kHz). Mas NÃO manda um datagrama por amostra: 1000
// pacotes minúsculos/s estouram o airtime do WiFi e a perda dispara (o
// histórico mostra 5–37% já a 100 Hz). Em vez disso agrupa BATCH_N amostras
// por datagrama → ~100 pacotes/s, taxa que o link sustenta. Cada amostra leva
// seu próprio seq e t_us (micros), então o receiver reconstrói o stream de
// 1 kHz, detecta perda por amostra e coloca tudo numa grade temporal comum.
#define SAMPLE_INTERVAL_US 1000          // 1 ms → 1 kHz
#define BATCH_N            10            // amostras por datagrama → 100 pkt/s
static uint32_t last_sample_us = 0;

struct __attribute__((packed)) Sample {
    uint32_t seq;       // contador por AMOSTRA
    uint32_t t_us;      // micros() no instante da amostra (relógio de sync)
    float    v_sensor;  // tensão calibrada, só com a média do oversampling
};

static Sample   batch[BATCH_N];
static uint8_t  batch_count = 0;
static uint32_t tx_seq = 0;

const uint32_t WIFI_RETRY_MS = 3000;
static uint32_t last_wifi_retry_ms = 0;

// Conexão inicial — só roda no setup(). Bloquear no boot é aceitável (não
// há amostragem ainda); na operação a reconexão é feita por wifi_kick().
static void wifi_connect()
{
    WiFi.mode(WIFI_STA);
    WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    // Desliga o modem-sleep: por padrão o ESP32 dorme o rádio entre DTIMs do AP
    // e atrasa/derruba pacotes — a 100 Hz isso aparecia como perda de UDP de
    // 5–37% no force_receiver. Sem sleep o rádio fica sempre ativo (consome mais
    // ~80 mA, irrelevante com alimentação USB/bancada) e a perda cai muito.
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);

    // Sem Serial: espera a conexão (ou desiste em 15 s e segue p/ reconexão
    // em background via wifi_kick). Nenhum print aqui para não acoplar o boot
    // a um terminal nem custar ciclos.
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(500);
    }
    last_wifi_retry_ms = millis();
}

// Atende o "hello" do force_receiver (auto-descoberta). Não-bloqueante: lê
// todos os datagramas pendentes na porta de descoberta e, se a tag bater,
// renova o slot do remetente em g_subs (ou ocupa um livre/expirado — na
// falta, rouba o mais antigo).
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
                if (slot < 0) slot = i;      // 1º livre/expirado
            }
            if (now - g_subs[i].last_hello_ms >
                now - g_subs[oldest].last_hello_ms) oldest = i;
        }
        if (slot < 0) slot = oldest;         // lotado: rouba o mais antigo
        g_subs[slot].ip            = src;
        g_subs[slot].last_hello_ms = now;
        g_subs[slot].used          = true;
    }
}

// Reconexão não-bloqueante: dispara um begin() throttled e retorna na hora.
static void wifi_kick()
{
    uint32_t now = millis();
    if (now - last_wifi_retry_ms < WIFI_RETRY_MS) return;
    last_wifi_retry_ms = now;
    WiFi.begin(ssid, password);
}

// ======================================================
// OTA — gravação pela rede (espota). A 1ª gravação ainda é por USB; depois
// é só `pio run -t upload` apontando para o IP do ESP. A senha evita que
// qualquer um na rede regrave o dispositivo.
// ======================================================
#define OTA_HOSTNAME  "forcedriver"
#define OTA_PASSWORD  "Biolabeb0608"

static void ota_setup()
{
    // Sem callbacks de Serial: a gravação OTA funciona igual; o progresso/erro
    // aparece no lado do pio (host), não na serial do ESP.
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
    // Serial DESLIGADA de propósito: nada de prints no caminho dos dados. O ADC,
    // o WiFi e o OTA não dependem da UART.
    analogReadResolution(12);
    analogSetPinAttenuation(ADC_PIN, ADC_11db);  // fundo de escala ~0..3.3 V
    wifi_connect();
    ota_setup();
    udpRx.begin(DISCOVERY_PORT);   // escuta o hello do force_receiver
    last_sample_us = micros();
}

// Monta o datagrama com as amostras acumuladas e o envia por unicast a CADA
// assinante fresco (broadcast no fallback, se não houver nenhum). Espelha o
// formato lido pelo force_receiver (LOAD_CELL_SAMPLE_FMT '<IIf',
// LOAD_CELL_BATCH_N amostras).
static void flush_batch()
{
    if (batch_count == 0) return;
    // millis() (não micros()/1000): last_hello_ms é medido em millis(); os
    // dois têm wrap diferente e não são comparáveis se misturados.
    uint32_t now = millis();
    bool sent = false;
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (!g_subs[i].used ||
            now - g_subs[i].last_hello_ms >= HELLO_TIMEOUT_MS) continue;
        udp.beginPacket(g_subs[i].ip, UDP_PORT);
        udp.write(reinterpret_cast<const uint8_t*>(batch),
                  batch_count * sizeof(Sample));
        udp.endPacket();
        sent = true;
    }
    if (!sent) {
        udp.beginPacket(BCAST_IP, UDP_PORT);
        udp.write(reinterpret_cast<const uint8_t*>(batch),
                  batch_count * sizeof(Sample));
        udp.endPacket();
    }
    batch_count = 0;
}

// ======================================================
// LOOP — 1 kHz não-bloqueante (amostra) + lote a ~100 Hz (envio)
// ======================================================
void loop()
{
    if (WiFi.status() != WL_CONNECTED) {
        wifi_kick();                     // não-bloqueante: tenta e segue
        last_sample_us = micros();       // evita rajada ao religar
        adc_sum_mv = 0; adc_count = 0;   // descarta acúmulo parcial
        batch_count = 0;                 // não envia lote meio montado
        return;
    }

    // Housekeeping (OTA + auto-descoberta) a ~20 Hz, FORA do caminho quente:
    // rodá-los a cada iteração (dezenas de milhares de vezes/s) só somava
    // latência/jitter ao laço de 1 kHz. 50 ms é folgado — o hello vem a cada 2 s
    // e o início de uma gravação OTA tolera dezenas de ms.
    static uint32_t last_house_ms = 0;
    uint32_t house_ms = millis();
    if (house_ms - last_house_ms >= 50) {
        last_house_ms = house_ms;
        ArduinoOTA.handle();
        discovery_poll();
    }

    // Acumula UMA leitura (mV calibrados) por passagem do loop(). Entre dois
    // ticks de 1 ms o loop roda ~15-30 vezes: é o oversampling LEVE que sobra a
    // 1 kHz e dá o dither sub-mV (o filtro pesado mora no PC agora).
    adc_sum_mv += (uint32_t)analogReadMilliVolts(ADC_PIN);
    adc_count++;

    // Subtração unsigned: trata o wrap do micros() (~71 min) corretamente.
    uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_sample_us) < SAMPLE_INTERVAL_US) return;
    if ((uint32_t)(now_us - last_sample_us) > 4 * SAMPLE_INTERVAL_US) {
        last_sample_us = now_us;         // ficou pra trás (WiFi/OTA) — re-ancora
    } else {
        last_sample_us += SAMPLE_INTERVAL_US;
    }

    if (adc_count == 0) return;   // segurança: nada acumulado neste tick
    // Média FRACIONÁRIA dos mV (sub-mV pelo dither) → volts.
    float v_adc = (adc_sum_mv / (float)adc_count) / 1000.0f;
    adc_sum_mv = 0; adc_count = 0;

    float v_sensor = v_adc * V_GAIN - V_OFFSET;

    // Enfileira a amostra no lote (filtro pesado fica no force_receiver).
    Sample& s = batch[batch_count];
    s.seq   = tx_seq++;
    s.t_us  = now_us;
#if DIAG_RAW
    s.v_sensor = v_adc;          // tensão CRUA do pino (sem nada)
#else
    s.v_sensor = v_sensor;       // leve: só a média do oversampling
#endif
    if (++batch_count >= BATCH_N) flush_batch();
    // (Sem aviso de saturação por Serial: a checagem do SPAN do ADC é feita na
    //  GUI/receiver pela tensão recebida — nada de prints no laço de amostragem.)
}
