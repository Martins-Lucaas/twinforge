"""
force_receiver_node.py — Recebe os pacotes UDP da ESP32 (célula de carga)
e publica os dados como tópicos ROS2.

Tópicos publicados:
  /load_cell/voltage     std_msgs/Float32   tensão FILTRADA (V)
  /load_cell/voltage_raw std_msgs/Float32   tensão CRUA (diagnóstico)
  /load_cell/force       std_msgs/Float32   força calibrada (N, compressão +)
  /load_cell/calibrated  std_msgs/Bool      True quando calibração existe

Calibração lida de lc_calib_read_path() (local > versionada no repo),
recarregada a cada 10 s. Calibração feita com OUTRA assinatura de firmware
(voltage_scale/voltage_offset ≠ constants) é RECUSADA: a tensão continua
fluindo, mas /load_cell/force não é publicada.

O firmware envia cada amostra num datagrama LOAD_CELL_SAMPLE_FMT '<IIf'
(seq, t_us, v_sensor). O filtro pesado e a conversão tensão→força são feitos
AQUI — reajustar filtro/calibração não exige reflashar a ESP.
"""

import json
import math
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool

from .constants import (
    LOAD_CELL_UDP_PORT as UDP_PORT,
    lc_calib_read_path,
    LOAD_CELL_SAMPLE_FMT as SAMPLE_FMT,
    LOAD_CELL_BATCH_N as BATCH_N,
    LOAD_CELL_ESP_IP as ESP_IP,
    LOAD_CELL_DISCOVERY_PORT as DISCOVERY_PORT,
    LOAD_CELL_DISCOVERY_MAGIC as DISCOVERY_MAGIC,
    LOAD_CELL_NOMINAL_RATE_HZ,
    LC_FW_VOLTAGE_SCALE,
    LC_FW_VOLTAGE_OFFSET,
)

SAMPLE_SZ = struct.calcsize(SAMPLE_FMT)   # 12 bytes

# ── Filtro pesado: mediana + One-Euro (Casiez et al. 2012) ───────────────────
# Cutoff ADAPTATIVO: parado cai a ONE_EURO_MINCUTOFF (zero firme); em
# movimento sobe e a latência despenca. O dt REAL vem do t_us do firmware,
# então a taxa do HX711 (10 vs 80 Hz) não precisa ser conhecida.
MEDIAN_N = 3                  # rejeita glitch isolado de 1 amostra
ONE_EURO_FREQ      = LOAD_CELL_NOMINAL_RATE_HZ   # chute até o 1º dt medido
ONE_EURO_MINCUTOFF = 4.0      # Hz — repouso (↓ = zero mais firme, +lag parado)
ONE_EURO_BETA      = 7.0      # responsividade ao movimento (↑ = menos lag, +ruído)
ONE_EURO_DCUTOFF   = 5.0      # Hz — cutoff do estimador de derivada


class _LoadCellFilter:
    """Mediana de MEDIAN_N seguida do One-Euro (passa-baixa adaptativo)."""

    def __init__(self, freq: float = ONE_EURO_FREQ,
                 mincutoff: float = ONE_EURO_MINCUTOFF,
                 beta: float = ONE_EURO_BETA,
                 dcutoff: float = ONE_EURO_DCUTOFF,
                 median_n: int = MEDIAN_N):
        self._freq = freq
        self._mincutoff = mincutoff
        self._beta = beta
        self._dcutoff = dcutoff
        self._median_n = median_n
        self._median_buf: list[float] = []
        self._mi = 0
        self._x_prev = 0.0
        self._dx_prev = 0.0
        self._seeded = False

    @staticmethod
    def _alpha(cutoff: float, freq: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * freq)

    def update(self, v: float, dt: float | None = None) -> float:
        """``dt`` = intervalo real desde a amostra anterior (s); None usa a
        taxa nominal."""
        freq = (1.0 / dt) if dt else self._freq
        if not self._seeded:
            self._median_buf = [v] * self._median_n
            self._x_prev = v
            self._dx_prev = 0.0
            self._seeded = True
            return v
        self._median_buf[self._mi] = v
        self._mi = (self._mi + 1) % self._median_n
        v_med = sorted(self._median_buf)[self._median_n // 2]
        dx = (v_med - self._x_prev) * freq
        a_d = self._alpha(self._dcutoff, freq)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev
        cutoff = self._mincutoff + self._beta * abs(dx_hat)
        a = self._alpha(cutoff, freq)
        x_hat = a * v_med + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        self._dx_prev = dx_hat
        return x_hat

QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class ForceReceiverNode(Node):

    def __init__(self):
        super().__init__('force_receiver')

        self._voltage_pub     = self.create_publisher(Float32, '/load_cell/voltage',     QOS_SENSOR)
        self._voltage_raw_pub = self.create_publisher(Float32, '/load_cell/voltage_raw', QOS_SENSOR)
        self._force_pub       = self.create_publisher(Float32, '/load_cell/force',       QOS_SENSOR)
        self._calib_pub   = self.create_publisher(Bool,    '/load_cell/calibrated', 10)

        # Sem calibração carregada (ou com assinatura divergente) a força NÃO
        # é publicada.
        self._slope:      float = 0.0
        self._intercept:  float = 0.0
        self._calibrated: bool  = False
        self._calib_warned: bool = False
        self._lock = threading.Lock()

        # Filtro pesado — só a thread UDP o toca, sem lock.
        self._filter = _LoadCellFilter()
        # t_us anterior, p/ medir o dt real do stream. Só a thread UDP toca.
        self._last_t_us: int | None = None

        # Detecção de perda de pacotes via seq da ESP32.
        self._last_seq:  int | None = None
        self._lost_pkts: int = 0
        self._rx_pkts:   int = 0
        self._seq_resets: int = 0
        self.create_timer(10.0, self._report_packet_loss)

        # Hello periódico ao ESP p/ receber a telemetria por unicast; se o
        # hello não chega, o firmware cai sozinho no broadcast.
        self._disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._send_discovery()
        self.create_timer(2.0, self._send_discovery)

        self._load_calib()
        self.create_timer(10.0, self._load_calib)
        self.create_timer(1.0, self._publish_calibrated)

        self._running = True
        self._udp_thr = threading.Thread(
            target=self._udp_loop, daemon=True, name='udp-force-rx')
        self._udp_thr.start()

        self.get_logger().info(
            f'ForceReceiver: UDP :{UDP_PORT} | calibrado={self._calibrated}')

    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _calib_signature_ok(data: dict) -> bool:
        """True se a calibração foi feita com a escala/offset de firmware
        VIGENTES. Campos ausentes contam como divergentes."""
        scale_raw  = data.get('voltage_scale')
        offset_raw = data.get('voltage_offset')
        if scale_raw is None or offset_raw is None:
            return False
        return (math.isclose(float(scale_raw), LC_FW_VOLTAGE_SCALE,
                             rel_tol=1e-6, abs_tol=1e-12)
                and math.isclose(float(offset_raw), LC_FW_VOLTAGE_OFFSET,
                                 rel_tol=1e-6, abs_tol=1e-9))

    def _load_calib(self) -> None:
        try:
            with open(lc_calib_read_path()) as f:
                data = json.load(f)
            sl = float(data['slope'])
            ic = float(data['intercept'])
            # Assinatura divergente → slope/intercept são de OUTRA escala de
            # tensão; publicar força com eles alimentaria a malha de controle
            # com valores errados. Recusa e pede recalibração.
            if not self._calib_signature_ok(data):
                if self._calibrated or not self._calib_warned:
                    self._calib_warned = True
                    self.get_logger().warn(
                        'Calibração ignorada: feita com firmware '
                        f"scale={data.get('voltage_scale')} "
                        f"offset={data.get('voltage_offset')}, mas o vigente é "
                        f'scale={LC_FW_VOLTAGE_SCALE:.6g} '
                        f'offset={LC_FW_VOLTAGE_OFFSET:.6g} — RECALIBRE na GUI '
                        '(a tensão continua sendo publicada normalmente).')
                with self._lock:
                    self._calibrated = False
                return
            with self._lock:
                changed = (sl != self._slope or ic != self._intercept
                           or not self._calibrated)
                self._slope      = sl
                self._intercept  = ic
                self._calibrated = True
                self._calib_warned = False   # rearma o aviso de assinatura
            if changed:
                self.get_logger().info(
                    f'Calibração carregada: slope={sl:.4f} intercept={ic:.6f}')
        except FileNotFoundError:
            pass
        except Exception as exc:
            self.get_logger().warn(f'Falha ao carregar calibração: {exc}')

    # ──────────────────────────────────────────────────────────────────
    def _send_discovery(self) -> None:
        try:
            self._disc_sock.sendto(DISCOVERY_MAGIC, (ESP_IP, DISCOVERY_PORT))
        except OSError:
            pass  # rede fora do ar: o firmware mantém broadcast no fallback

    # ──────────────────────────────────────────────────────────────────
    def _publish_calibrated(self) -> None:
        with self._lock:
            is_cal = self._calibrated
        msg = Bool(); msg.data = is_cal
        self._calib_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    def _udp_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)  # múltiplos nós no mesmo PC
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Buffer generoso: se a thread engasgar por alguns ms o kernel não
        # descarta datagramas.
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        sock.settimeout(1.0)
        try:
            sock.bind(('', UDP_PORT))
        except OSError as exc:
            self.get_logger().error(
                f'Bind UDP :{UDP_PORT} falhou: {exc}')
            return
        self.get_logger().info(f'UDP bind OK em 0.0.0.0:{UDP_PORT} (broadcast)')

        rcvbuf = SAMPLE_SZ * BATCH_N + 64
        while self._running and rclpy.ok():
            try:
                raw, _ = sock.recvfrom(max(256, rcvbuf))
            except socket.timeout:
                continue
            except OSError:
                break

            # Datagrama = N amostras concatenadas; ignora rabo truncado.
            n_samples = len(raw) // SAMPLE_SZ
            if n_samples == 0:
                continue

            with self._lock:
                sl  = self._slope
                ic  = self._intercept
                cal = self._calibrated

            for k in range(n_samples):
                off = k * SAMPLE_SZ
                (seq, t_us, v_raw) = struct.unpack_from(SAMPLE_FMT, raw, off)
                self._track_seq(seq)

                vr_msg = Float32(); vr_msg.data = float(v_raw)
                self._voltage_raw_pub.publish(vr_msg)

                # dt real pelo relógio do firmware (wrap de uint32 tratado).
                # Fora de (0, 0.5 s] — 1ª amostra, reset da ESP ou pacote fora
                # de ordem — cai na taxa nominal do filtro.
                dt = None
                if self._last_t_us is not None:
                    d_us = (t_us - self._last_t_us) & 0xFFFFFFFF
                    if 0 < d_us <= 500_000:
                        dt = d_us / 1e6
                self._last_t_us = t_us

                v_sensor = self._filter.update(float(v_raw), dt)

                v_msg = Float32(); v_msg.data = v_sensor
                self._voltage_pub.publish(v_msg)

                if cal and abs(sl) > 1e-9:
                    # Calibração feita em tração → invertida p/ a convenção do
                    # sistema: compressão = positivo.
                    force = (ic - v_sensor) / sl
                    f_msg = Float32(); f_msg.data = float(force)
                    self._force_pub.publish(f_msg)

        sock.close()

    # ──────────────────────────────────────────────────────────────────
    # Salto de seq acima disto não é perda plausível: re-ancora sem contar.
    _MAX_PLAUSIBLE_GAP = 500

    def _track_seq(self, seq: int) -> None:
        """Contabiliza pacotes perdidos pelo salto do seq. Delta calculado
        COM SINAL: um reset da ESP (seq volta a 0 após boot/OTA) dá delta
        negativo → re-ancora, não conta como perda. Roda na thread UDP; os
        contadores são lidos pelo timer em outra thread → mesmo lock."""
        with self._lock:
            self._rx_pkts += 1
            if self._last_seq is not None:
                delta = seq - self._last_seq
                if delta <= 0:
                    self._seq_resets += 1
                elif delta <= self._MAX_PLAUSIBLE_GAP:
                    self._lost_pkts += delta - 1
            self._last_seq = seq

    def _report_packet_loss(self) -> None:
        with self._lock:
            rx, lost, resets = self._rx_pkts, self._lost_pkts, self._seq_resets
            self._rx_pkts = 0
            self._lost_pkts = 0
            self._seq_resets = 0
        if resets:
            self.get_logger().info(
                f'ESP32 reiniciou {resets}× nos últimos 10 s (seq reancorado) — '
                'provável boot/OTA, não é perda de rede.')
        if rx == 0 or lost == 0:
            return
        total = rx + lost
        pct = 100.0 * lost / total if total else 0.0
        self.get_logger().warn(
            f'Perda de pacotes UDP: {lost}/{total} '
            f'({pct:.1f}%) nos últimos 10 s')

    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        try:
            self._disc_sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForceReceiverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
