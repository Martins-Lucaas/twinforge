"""
palpation_logger.py — Nó ROS 2 que grava cada execução de palpação em disco.

Lê cinco tópicos:
  sub /palpation/start     touch_pack_msgs/PalpationStart — parâmetros do
                                               experimento; marca o início de
                                               um "run".
  sub /palpation/status    touch_pack_msgs/PalpationStatus — fase e ciclo
                                               atuais; encerra o run em
                                               DONE/ABORTED.
  sub /load_cell/force_net std_msgs/Float32  força tare-compensada (N,
                                               compressão positiva) — é o sinal
                                               CANÔNICO do experimento e o
                                               gatilho de amostragem (~50 Hz).
  sub /touch_sensor/adc    std_msgs/Int32MultiArray — frame de taxels (ADC) do
                                               STM32, republicado pela GUI; o
                                               ÚLTIMO frame é copiado em cada
                                               amostra de força (colunas taxel_*).
  sub /touch_sensor/spike_event std_msgs/String — um evento por mensagem
                                               (RA|SA|CN_MM|CN_RA|CN_SA); contados
                                               POR amostra (colunas n_RA/n_SA/
                                               cn_mm/cn_ra/cn_sa).
  sub /joint_states        sensor_msgs/JointState — juntas do braço; a pose do
                                               TCP é calculada via FK
                                               (kinematics + T_TOUCH_TOOL_ATTACH).

Saída (em sensors/Data/):
  <timestamp>__samples.csv   uma linha por amostra de força (~1 kHz):
      t_rel_s, t_unix, cycle, phase (CÓDIGO numérico — ver PHASE_CODES),
      setpoint_n, force_net_n, q1..q6, tcp_x, tcp_y, tcp_z,
      taxel_0..taxel_N (último frame ADC), n_RA, n_SA, cn_mm, cn_ra, cn_sa
  <timestamp>__params.json   parâmetros do start
  <timestamp>__summary.json  métricas pós-run (gerado pelo palpation_report)
  <timestamp>__plot.png      força×tempo por fase (se matplotlib disponível)

Encerramento: o run fecha quando recebe status DONE ou ABORTED, ou após
5 min sem amostras de força (timeout de segurança caso o explorer caia).
Ao fechar um run com amostras, o relatório é gerado automaticamente em
background (ver palpation_report.generate_report).
"""
from __future__ import annotations

import csv
import json
import logging
import os
import threading
import time
from datetime import datetime
from typing import IO

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from std_msgs.msg import Float32, String, Int32MultiArray
from sensor_msgs.msg import JointState
from rosidl_runtime_py.convert import message_to_ordereddict
from touch_pack_msgs.msg import PalpationStart, PalpationStatus

from .kinematics import forward_kinematics, T_TOUCH_TOOL_ATTACH
from .constants import (
    ARM_JOINTS as _ARM_JOINTS,
    RUNS_DIR as OUTPUT_DIR,
    TOUCH_ADC_TOPIC, TOUCH_EVENT_TOPIC,
    TOUCH_TAXELS_DEFAULT as _N_TAXELS, TOUCH_EVENT_TYPES,
    PHASE_CODES,
)


log = logging.getLogger('touch_pack.palpation_logger')

RUN_IDLE_TIMEOUT_S = 300.0   # 5 min sem força → fecha run "perdido"

# CSV unificado do experimento (1 linha por amostra de força, ~1 kHz):
#   tempo + ciclo + fase NUMÉRICA + setpoint + força + juntas + TCP +
#   25 taxels (último frame ADC) + contagem de eventos por amostra.
# Colunas de eventos: n_RA/n_SA (spikes) e cn_mm/cn_ra/cn_sa (cuneiformes).
_EVENT_COLS = ['n_RA', 'n_SA', 'cn_mm', 'cn_ra', 'cn_sa']
# Ordem que parea TOUCH_EVENT_TYPES → _EVENT_COLS (RA,SA,CN_MM,CN_RA,CN_SA).
CSV_HEADER = (['t_rel_s', 't_unix', 'cycle', 'phase', 'setpoint_n', 'force_net_n']
              + [f'q{i}' for i in range(1, 7)]
              + ['tcp_x', 'tcp_y', 'tcp_z']
              + [f'taxel_{i}' for i in range(_N_TAXELS)]
              + _EVENT_COLS)

_QOS_COMMAND = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class PalpationLogger(Node):

    def __init__(self):
        super().__init__('palpation_logger')

        os.makedirs(OUTPUT_DIR, exist_ok=True)

        self._lock = threading.Lock()
        self._csv_fh: IO | None = None
        self._csv_writer: csv.writer | None = None
        self._run_path: str | None = None
        self._run_t0: float | None = None
        self._phase: str = 'IDLE'
        self._cycle: int = 0
        self._last_sample_t: float = 0.0
        self._sample_count: int = 0
        # Últimas juntas do braço (rad, ordem _ARM_JOINTS); None até o
        # primeiro /joint_states. _q_cols / _tcp_cols são as colunas do CSV já
        # formatadas — a FK é cara e roda agora a cada /joint_states (~50 Hz),
        # NÃO por amostra de força (a 1 kHz seriam 1000 FKs/s do mesmo valor).
        self._q: np.ndarray | None = None
        self._q_cols: list[str] = [''] * 6
        self._tcp_cols: list[str] = [''] * 3
        # Setpoint de força do run (force_n do start) — vai numa coluna do CSV.
        self._setpoint: float | None = None
        # Tátil completo (republicado pela GUI). _adc_cols = último frame ADC já
        # formatado (25 colunas); _evt_counts conta eventos POR amostra (zerado
        # a cada linha escrita no _cb_force).
        self._adc_cols: list[str] = [''] * _N_TAXELS
        self._evt_counts: dict[str, int] = {t: 0 for t in TOUCH_EVENT_TYPES}

        self.create_subscription(
            PalpationStart, '/palpation/start', self._cb_start, _QOS_COMMAND)
        self.create_subscription(
            PalpationStatus, '/palpation/status', self._cb_status, 10)
        self.create_subscription(
            Float32, '/load_cell/force_net', self._cb_force, _QOS_SENSOR)
        self.create_subscription(
            Int32MultiArray, TOUCH_ADC_TOPIC, self._cb_adc, _QOS_SENSOR)
        self.create_subscription(
            String, TOUCH_EVENT_TOPIC, self._cb_event, _QOS_SENSOR)
        self.create_subscription(
            JointState, '/joint_states', self._cb_joints, 50)

        # Watchdog @1 Hz para fechar runs órfãos.
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f'palpation_logger ativo — gravando em {OUTPUT_DIR}/')

    # ── Callbacks ────────────────────────────────────────────────────
    def _cb_start(self, msg: PalpationStart) -> None:
        """Início de um novo run: cria CSV e dump dos parâmetros em JSON.
        O msg tipado vira dict (mesmas chaves dos campos) para o
        __params.json — o palpation_report lê 'force_n' etc. de lá."""
        try:
            params = dict(message_to_ordereddict(msg))
            params['home_deg'] = list(params.get('home_deg', []))
        except Exception:
            params = {}
        try:
            setpoint = float(getattr(msg, 'force_n'))
        except (AttributeError, TypeError, ValueError):
            setpoint = None

        with self._lock:
            self._close_run_locked('superseded')
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_path = os.path.join(OUTPUT_DIR, f'{ts}__samples.csv')
            json_path = os.path.join(OUTPUT_DIR, f'{ts}__params.json')
            fh = None
            try:
                fh = open(csv_path, 'w', newline='')
                writer = csv.writer(fh)
                writer.writerow(CSV_HEADER)
                with open(json_path, 'w') as pf:
                    json.dump(params, pf, indent=2, sort_keys=True)
            except OSError as exc:
                self.get_logger().error(
                    f'Falha ao criar arquivos de run: {exc}')
                if fh is not None:
                    fh.close()
                return
            self._csv_fh = fh
            self._csv_writer = writer
            self._run_path = csv_path
            self._run_t0 = time.time()
            self._last_sample_t = self._run_t0
            self._sample_count = 0
            self._phase = 'IDLE'
            self._cycle = 0
            self._setpoint = setpoint
            self._adc_cols = [''] * _N_TAXELS
            self._evt_counts = {t: 0 for t in TOUCH_EVENT_TYPES}
            self.get_logger().info(
                f'Run iniciado → {os.path.basename(csv_path)} '
                f'(setpoint={setpoint if setpoint is not None else "?"} N)')

    def _cb_status(self, msg: PalpationStatus) -> None:
        with self._lock:
            self._phase = msg.phase
            self._cycle = int(msg.cycle)
            if msg.phase in ('DONE', 'ABORTED') and self._csv_fh is not None:
                self._close_run_locked(msg.phase)

    def _cb_adc(self, msg: Int32MultiArray) -> None:
        """Frame de taxels (ADC) republicado pela GUI. Guarda as colunas já
        formatadas (ajustadas a _N_TAXELS: trunca/preenche) — cada amostra de
        força copia o ÚLTIMO frame recebido."""
        vals = list(msg.data)
        cols = [str(int(v)) for v in vals[:_N_TAXELS]]
        if len(cols) < _N_TAXELS:
            cols += [''] * (_N_TAXELS - len(cols))
        with self._lock:
            self._adc_cols = cols

    def _cb_event(self, msg: String) -> None:
        """Um spike/cuneiforme (tipo em msg.data). Conta por tipo; o contador é
        zerado a cada linha do CSV (contagem POR amostra de força)."""
        t = str(msg.data)
        with self._lock:
            if t in self._evt_counts:
                self._evt_counts[t] += 1

    def _cb_joints(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in _ARM_JOINTS):
            return   # mensagem só com juntas da mão
        q = np.array([float(msg.position[idx[j]]) for j in _ARM_JOINTS])
        # FK aqui (~50 Hz), não no _cb_force (1 kHz): as colunas ficam prontas
        # e cada amostra de força só as copia.
        q_cols = [f'{v:.5f}' for v in q]
        try:
            tcp = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
            tcp_cols = [f'{v:.5f}' for v in tcp]
        except Exception:
            tcp_cols = ['', '', '']
        with self._lock:
            self._q = q
            self._q_cols = q_cols
            self._tcp_cols = tcp_cols

    def _cb_force(self, msg: Float32) -> None:
        """Uma amostra por leitura de força (agora ~1 kHz) — sinal canônico.
        Cada amostra vira uma linha do CSV unificado, então o arquivo sai na
        taxa da força; com a ESP a 1 kHz é um log força+toque a 1 kHz."""
        now = time.time()
        with self._lock:
            self._last_sample_t = now
            if self._csv_writer is None or self._run_t0 is None:
                return
            # Colunas de junta/TCP já calculadas no _cb_joints (FK fora daqui).
            q_cols = self._q_cols
            tcp_cols = self._tcp_cols
            # Tátil: último frame ADC + contagem de eventos DESDE a amostra
            # anterior (zerada após escrever). Fase como código numérico.
            adc_cols = self._adc_cols
            evt_cols = [self._evt_counts[t] for t in TOUCH_EVENT_TYPES]
            self._evt_counts = {t: 0 for t in TOUCH_EVENT_TYPES}
            phase_code = PHASE_CODES.get(self._phase, -1)
            setpoint = ('' if self._setpoint is None
                        else f'{self._setpoint:.4f}')
            try:
                self._csv_writer.writerow([
                    f'{now - self._run_t0:.4f}',
                    f'{now:.4f}',
                    self._cycle,
                    phase_code,
                    setpoint,
                    f'{float(msg.data):.4f}',
                    *q_cols,
                    *tcp_cols,
                    *adc_cols,
                    *evt_cols,
                ])
                self._sample_count += 1
                # Flush a cada ~1 s (1000 amostras @ 1 kHz) para não perder
                # dados se o nó for morto sem encerrar limpo, sem martelar o
                # disco a cada amostra.
                if self._sample_count % 1000 == 0 and self._csv_fh is not None:
                    self._csv_fh.flush()
            except (ValueError, OSError) as exc:
                self.get_logger().warn(f'Falha ao gravar amostra: {exc}')

    def _watchdog(self) -> None:
        with self._lock:
            if self._csv_fh is None or self._run_t0 is None:
                return
            if time.time() - self._last_sample_t > RUN_IDLE_TIMEOUT_S:
                self.get_logger().warn(
                    f'Run sem amostras de força há {RUN_IDLE_TIMEOUT_S:.0f}s '
                    '— encerrando por timeout.')
                self._close_run_locked('timeout')

    # ── Encerramento ─────────────────────────────────────────────────
    def _close_run_locked(self, reason: str) -> None:
        """Fecha o run atual. Deve ser chamado com `self._lock`."""
        if self._csv_fh is None:
            return
        try:
            self._csv_fh.flush()
            self._csv_fh.close()
        except OSError:
            pass
        duration = (time.time() - self._run_t0
                     if self._run_t0 else 0.0)
        run_path = self._run_path
        n_samples = self._sample_count
        self.get_logger().info(
            f'Run encerrado ({reason}): {n_samples} amostras '
            f'em {duration:.1f}s → {os.path.basename(run_path or "?")}')
        self._csv_fh = None
        self._csv_writer = None
        self._run_path = None
        self._run_t0 = None
        self._sample_count = 0
        # Relatório pós-run (summary JSON + gráfico) em background — só
        # para runs concluídos com dados; 'superseded' é um run substituído.
        if run_path and n_samples > 0 and reason != 'superseded':
            threading.Thread(
                target=self._generate_report, args=(run_path,),
                daemon=True, name='palpation-report').start()

    def _generate_report(self, csv_path: str) -> None:
        try:
            from .palpation_report import generate_report
            summary = generate_report(csv_path)
            self.get_logger().info(
                'Relatório gerado: '
                f'{os.path.basename(summary.get("summary_path", "?"))}')
        except Exception as exc:   # nunca derruba o logger
            self.get_logger().warn(f'Falha ao gerar relatório: {exc}')

    def close(self) -> None:
        with self._lock:
            self._close_run_locked('shutdown')


def main(args=None):
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s.%(msecs)03d [%(name)s] %(levelname)s  %(message)s',
        datefmt='%H:%M:%S')
    rclpy.init(args=args)
    node = PalpationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
