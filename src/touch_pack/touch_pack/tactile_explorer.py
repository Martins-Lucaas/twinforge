"""
tactile_explorer.py — Backend ROS 2 da célula de palpação tátil.

Coreografia (Gupta et al., 2021) sobre uma SUPERFÍCIE HORIZONTAL.

    IDLE  →  HOME  →  DESCENDING  →  HOLD  →  SLIDING  →  HOME  →  IDLE

Arquitetura de controle:
  Todas as fases de movimento usam STREAMING DIRETO de setpoints a 33 Hz
  (publicação em /cr10_group_controller/joint_trajectory, 1 ponto por
  mensagem). Não há action server nem trajetórias pré-planejadas:
  cada passo é calculado e enviado individualmente no loop de controle.

  Vantagens:
    - Sem fila de movimentos acumulada no controlador.
    - Nenhum movimento residual de uma fase carrega para a próxima
      (_settle() publica a posição atual repetidamente antes de toda
      transição, zerando qualquer lookahead pendente).
    - Velocidade explicitamente limitada pelo tamanho do passo
      (step_m = v_ms × dt), independente do SpeedFactor do controlador.

Fases:
  HOME         Interpolação linear no espaço de juntas a ≤ 0.3 rad/s.
  DESCENDING   Aproxima rápido até o contato; então passo DEADBEAT
               normalizado pela rigidez leva a compressão ao setpoint
               (profundidade da GUI = curso máximo).
  HOLD         Passo DEADBEAT mantém a compressão no setpoint, ESPERA a
               janela estável e mantém um dwell de medição antes de liberar.
  SLIDING      Streaming Jacobiano lateral com ALTURA TRAVADA em posição —
               a força fica livre para variar com a textura (sinal medido).

Controle de força (DESCENDING/HOLD):
  Setpoint selecionável na GUI (force_n, máx. 10 N). A correção usa o passo
  DEADBEAT Δx=relax·(setpoint−fz)/K_est, com K_est=ΔF/Δx estimado online no
  DESCENDING e congelado para o HOLD (ver _StiffnessEstimator). No SLIDING
  a força NÃO é regulada — só monitorada por segurança. A medição é
  CANCELADA se a compressão exceder 15 N (_FORCE_ABORT_LIMIT_N).

Interface ROS:
  sub /palpation/start    touch_pack_msgs/PalpationStart
  sub /palpation/stop     std_msgs/String
  sub /palpation/pause    std_msgs/Bool     true=pausa (segura posição), false=retoma
  sub /load_cell/force_net std_msgs/Float32
  sub /joint_states       sensor_msgs/JointState
  pub /palpation/status   touch_pack_msgs/PalpationStatus
  pub /cr10_group_controller/joint_trajectory  (streaming direto)

Parâmetros ROS:
  approach_v_max_mms   50.0   velocidade inicial da descida (mm/s)
  approach_v_min_mms    5.0   velocidade final da descida (mm/s)
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time

import numpy as np
if tuple(int(x) for x in np.__version__.split(".")[:2]) >= (2, 0):
    sys.exit(
        f"[ERRO] NumPy {np.__version__} detectado — ABI incompatível com "
        "ROS 2 Humble.\n"
        "Corrija: pip install 'numpy<2'\n"
        "Confirme com: python3 -c \"import numpy; print(numpy.__version__)\""
    )
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

_QOS_COMMAND = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from touch_pack_msgs.msg import PalpationStart, PalpationStatus

from .kinematics import (
    forward_kinematics, jacobian,
    JOINT_MIN, JOINT_MAX, MIMIC_LIST as _MIMIC_LIST,
    T_TOUCH_TOOL_ATTACH,
)
from .constants import (
    ARM_JOINTS as _ARM_JOINTS,
    HAND_JOINTS as _HAND_PRIMARY,
    HAND_POINTING_RAD as _HAND_POINTING_RAD,
    POINTING_SEED_DEG as _POINTING_SEED_DEG,
    FORCE_ABORT_LIMIT_N as _FORCE_ABORT_LIMIT_N,
    FORCE_SETPOINT_MAX_N as _FORCE_SETPOINT_MAX_N,
)

_POINTING_SEED_Q = np.array(
    [math.radians(_POINTING_SEED_DEG[j]) for j in _ARM_JOINTS])

# ── Célula de carga calibrada (/load_cell/force_net) ─────────────────────────
# Convenção de sinal: compressão = POSITIVO, tração = NEGATIVO.
# O force_receiver_node publica força calibrada (N). A GUI aplica tara
# (tare_v - v_now) / slope → compressão positiva, tração negativa.
_SLIDING_SAFETY_M   = 0.30   # m: distância máxima de segurança no SLIDING

# ── Teto absoluto de força aplicada ──────────────────────────────────────────
# REQUISITO: o sistema NUNCA pode exceder _FORCE_ABORT_LIMIT_N (15 N) de força
# aplicada. Como a leitura tem atraso (filtro) e o braço tem inércia, dispara-se
# a parada com MARGEM, em _FORCE_SAFE_LIMIT_N, para que o overshoot residual não
# ultrapasse os 15 N. Todas as fases (DESCENDING/HOLD/SLIDING) checam a força
# ANTES de comandar movimento e, ao cruzar a margem, travam a posição na hora.
_FORCE_SAFE_LIMIT_N = 12.0   # N: margem de 3 N abaixo do teto de 15 N

# ── PID de força (DESCENDING / HOLD / SLIDING) ───────────────────────────────
_PID_V_MAX_MS = 0.005    # m/s: correção máxima do PID (5 mm/s)
_PID_I_MAX_Ns = 5.0      # N·s: anti-windup do integrador

# ── Estimador de rigidez do contato (usado pela regulação quase-estática) ────
# O micro-passo de força é normalizado pela rigidez: Δx = relax·(alvo−fz)/K.
# K_est = ΔF/Δx é estimada online durante o contato (_StiffnessEstimator);
# no modo quase-estático os pares (Δx, ΔF) são medidos em REPOUSO — sem o
# erro de fase que contaminava a estimativa contínua.
_K_DEFAULT_NM      = 40_000.0   # N/m (40 N/mm): rigidez default antes de estimar
_K_MIN_NM          =  8_000.0   # N/m (8 N/mm):  piso do estimador
_K_MAX_NM          = 1_000_000.0  # N/m (1000 N/mm): teto do estimador —
                                  # o sensor bem fixo mediu ~900 N/mm
                                  # (02/07: 9,1 N por 10 µm); com teto de
                                  # 250 os passos saíam 3–4× grandes demais
_K_EMA_ALPHA       = 0.25       # filtro EMA do estimador de rigidez
_DEADBEAT_DX_MAX_M = 2.5e-4     # 0.25 mm: passo cheio do alívio de EMERGÊNCIA
                                # (_relieve_contact, ≈ 8 mm/s)
# Afundamento máximo do TCP abaixo do plano inicial durante o SLIDING. Se o
# deslize sair da amostra (borda), a correção de força empurraria para baixo
# indefinidamente atrás do setpoint; ao exceder este curso o deslize termina.
_SLIDE_MAX_SINK_M    = 0.010    # 10 mm
_CONTACT_ON_N      = 0.05       # N: força que caracteriza contato. A célula tem
                                # ruído ZERO em ar livre (coletas de 04/07) e
                                # degrau de leitura ~9 mN; 0.05 N detecta o toque
                                # com ~3× menos penetração que os 0.15 N antigos.
# Teto de segurança da velocidade de aproximação até o contato (m/s). Bound do
# transiente de impacto (≈ v·latência·K) sob a margem de força. Com o sensor
# rígido a rigidez medida chegou a ~150 N/mm (coleta de 02/07: 0,12 mm → 12 N;
# a 2 mm/s com ~50 ms de latência da malha o pico foi 20 N). A 1,5 mm/s o
# mesmo impacto fica ≈ 11 N, e o alívio assimétrico (recuo a passo cheio)
# derruba a força em 1–2 ticks. Reduza mais para amostras ainda mais rígidas.
# 04/07: a 1,0 mm/s o 1º impacto cravou 15,0 N (abort). Com o modo
# quase-estático o custo do impacto caiu para a detecção (~v·lat·K), então o
# teto caiu para 0,5 mm/s: impacto esperado 2–7 N nas rigidezes medidas.
_DESCEND_CONTACT_V_MAX_MS = 0.0005   # 0,5 mm/s
# Descida em DOIS ESTÁGIOS: a profundidade do 1º contato é memorizada
# (_learned_contact_m) — o alvo não muda entre toques — e as descidas
# seguintes vão à velocidade cheia da GUI até a margem antes do ponto
# aprendido, rastejando só no trecho final. Se a superfície subir mais que a
# margem (fixação mexida), o impacto em velocidade cheia dispara a cadeia de
# segurança (_relieve_contact + abort) — falha ruidosa, não silenciosa.
_CONTACT_ZONE_MARGIN_M = 0.0015   # 1,5 mm: zona lenta antes do contato aprendido
                                  # (era 3 mm; com rastejo mais lento a margem
                                  # menor mantém o tempo da zona ~7,5 s)
_DESCEND_TOUCH_V_MS    = 0.0002   # 0,2 mm/s: rastejo final com zona aprendida.
                                  # A 0,5 mm/s o toque ainda gerava 5–10 N
                                  # (04/07, K_eff 40–110 N/mm); a 0,2 mm/s o
                                  # transiente v·lat·K fica ~0,8–2 N — do mesmo
                                  # tamanho do setpoint típico.

# ── Regulação QUASE-ESTÁTICA de força (move-then-measure) ────────────────────
# A malha contínua mede em MOVIMENTO com ~3 ticks de atraso (One-Euro ~40 ms +
# execução JTC 1–2 ticks): contra contato rígido (40–110 N/mm nas coletas de
# 04/07; até ~900 N/mm em 02/07) todo passo comandado com leitura defasada
# alivia/aprofunda 3× demais e vira quique — 04/07 14:12–14:26: 83 re-impactos
# 0→12 N em 27 s, abort em 15 N, 0 s dentro da banda. No modo quase-estático o
# braço NUNCA se move durante a medição: congela _QS_SETTLE_TICKS (o pipeline
# de força esvazia), lê a mediana de _QS_MEDIAN_N amostras, decide UM
# micro-passo com ΔF projetado ≤ _QS_DF_MAX_N, executa em 1 tick e mede de
# novo. Sem lag não há ciclo-limite, e os pares (Δx, ΔF) medidos em REPOUSO
# tornam o K_est confiável de verdade. Dentro da banda não se move — a posição
# congelada É o hold. Ciclo ≈ 180 ms; do 1º toque ao setpoint ~4–8 ciclos.
_QS_SETTLE_TICKS   = 5       # ticks congelado antes de medir (150 ms > lag)
_QS_MEDIAN_N       = 3       # amostras da mediana settled
_QS_RELAX          = 0.7     # sub-relaxação do passo (robustez a erro de K_est)
_QS_DF_MAX_N       = 0.4     # N: ΔF projetado máximo por micro-passo
_QS_DX_MAX_M       = 2.0e-5  # 20 µm: teto absoluto do micro-passo
_QS_DX_PROBE_M     = 3.0e-6  # 3 µm: teto ANTES do 1º K_est settled — a
                             #   900 N/mm projeta ≤ 2,7 N (sob a margem)
_QS_FREE_STEP_M    = 5.0e-6  # 5 µm/ciclo: re-aproximação se perder contato
_QS_RELIEF_FLOOR_N = 0.10    # N: alívio nunca projeta abaixo disso — não
                             #   perde o contato (o alívio de passo cheio era
                             #   o que arremessava a ferramenta da superfície)
_QS_DF_DEAD_N      = 0.05    # N: ΔF mínimo p/ considerar que o passo "pegou";
                             #   abaixo (stiction/LSB da junta) o próximo
                             #   passo cresce 1,5× até o atuador responder
_QS_BOOST_MAX      = 6.0     # teto do multiplicador anti-stiction. Sem teto,
                             #   coletas de 04/07 14:51/14:52: boost cresceu até
                             #   o passo acumulado estourar 7–9 N de uma vez
_QS_DF_HARD_N      = 0.6     # N: teto DURO de ΔF projetado por passo, boost
                             #   INCLUSO — se a resolução do atuador exigir
                             #   mais que isso, estaciona e dá timeout (limite
                             #   físico; complacência mecânica é a saída)
_QS_DX_PROBE_MAX_M = 8.0e-6  # 8 µm: teto absoluto do passo-sonda mesmo com
                             #   boost (K ainda desconhecida → teto de ΔF
                             #   projetado não protege)
_QS_FREE_STEP_MAX_M = 8.0e-6 # 8 µm: teto da re-aproximação sem contato mesmo
                             #   com boost — limita o transiente de re-toque
_QS_ARRIVE_S       = 0.35    # s: janela settled em banda p/ o DESCENDING
                             #   declarar chegada e entregar ao HOLD
_QS_TIMEOUT_S      = 12.0    # s: teto da convergência inicial no DESCENDING
# Creep/relaxação viscoelástica: nas coletas de 04/07 14:51–14:52 a força a
# posição CONSTANTE relaxa até ~−1,2 N/s (τ ≈ 2–4 s). Dentro da banda o hold
# não pode só congelar — o creep arrasta a força para fora pela borda de
# baixo. Correção: dentro da banda, se |err| passar de meia-banda, aplica-se
# micro-passo de perseguição SEM resetar a janela de estabilidade (a banda é
# critério de força, não de imobilidade).

# ── Estabilização do setpoint no HOLD ────────────────────────────────────────
# O HOLD só libera o SLIDING quando a compressão fica DENTRO da tolerância
# em torno do setpoint por _HOLD_STABLE_S contínuos (tol = máx(_HOLD_TOL_N,
# _HOLD_TOL_PCT × setpoint)). Sair da banda reinicia a janela. Se não
# estabilizar em _HOLD_TIMEOUT_S, prossegue com aviso — o PID do SLIDING
# continua corrigindo a força durante o deslizamento.
_HOLD_TOL_N     = 0.15   # N: tolerância absoluta mínima (≈ ruído da célula)
_HOLD_TOL_PCT   = 0.05   # fração do setpoint (5 %)
_HOLD_STABLE_S  = 5.0    # s contínuos dentro da tolerância (janela estável)
_HOLD_TIMEOUT_S = 8.0    # s: teto de espera pela estabilização
# Após estabilizar, mantém o setpoint por mais _HOLD_DWELL_S (medição) antes de
# liberar SLIDING/recuo. Janela estável (~5 s) + dwell (~5 s) = toque rápido e
# repetível, sem a espera longa que o ganho fixo exigia.
_HOLD_DWELL_S   = 5.0

# ── Staleness da célula de carga ─────────────────────────────────────────────
# Idade máxima da última leitura de /load_cell/force_net para que o controle
# por força seja confiável. Se a ESP32/receiver cair no meio de uma fase
# controlada por força, _fz_corrected() devolveria um valor CONGELADO e o
# PID continuaria corrigindo às cegas (abaixo do setpoint → afundaria a
# ferramenta na mesa). DESCENDING/HOLD/SLIDING abortam com outcome 'stale'.
_FORCE_STALE_S = 0.5


# ── Parâmetros do loop de streaming ──────────────────────────────────────────
_CTRL_DT    = 0.030   # período de cada passo (33 Hz)
_CTRL_LOOK  = 0.10    # time_from_start do _settle (s)
_CTRL_WIN   = 10      # waypoints por batch de streaming (10 × 30 ms = 300 ms)
_SLIDE_WIN  = 3       # janela do SLIDING com PID (3 × 30 ms = 90 ms lookahead)
_JAC_LAM    = 0.01    # regularização DLS
_ORI_GAIN   = 0.5     # ganho de correção de orientação
_Z_CORR_GAIN = 0.5   # ganho de correção perpendicular durante sliding
_HOME_MAX_RAD_S = 0.05  # velocidade máxima do HOME (≈ 3°/s por junta);
                        # ajustável via parâmetro ROS home_speed_rad_s
_SETTLE_TICKS   = 6     # ticks de espera entre fases (6 × 30 ms = 180 ms)

# Velocidade máxima de referência (rad/s) por junta — equivale ao limite
# físico do CR10 (≈ 180°/s). O speed_factor_pct da GUI escala este valor:
# 10 % → 0.314 rad/s ≈ 18°/s (seguro para palpação).
_MAX_JOINT_VEL_RAD_S = math.pi  # 180°/s


class _ForcePID:
    """PID de força → velocidade de correção ao longo do approach (m/s).

    Convenção: erro = setpoint − compressão medida. Saída positiva
    aprofunda (mais compressão); negativa alivia. O integrador só
    acumula após o primeiro contato (evita windup durante aproximação
    sem carga) e é saturado em ±_PID_I_MAX_Ns. A saída é limitada a
    ±_PID_V_MAX_MS.
    """

    def __init__(self, kp: float, ki: float, kd: float, dt: float):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.ever_in_contact = False
        self._integral = 0.0
        self._prev_err = 0.0

    def step(self, err: float, in_contact: bool) -> float:
        if in_contact:
            self.ever_in_contact = True
        # Anti-windup na PERDA de contato: integra SÓ enquanto há contato.
        # Fora de contato o erro fica positivo (alvo − 0) e, se continuasse
        # acumulando, o integrador "enrolava" empurrando para baixo — ao
        # rebater a superfície isso virava overshoot e alimentava o ciclo
        # de quicar. Congelado fora de contato, o termo P (kp·err) ainda
        # reaproxima; o integral retoma quando o contato volta.
        if in_contact:
            self._integral = float(np.clip(
                self._integral + err * self.dt,
                -_PID_I_MAX_Ns, _PID_I_MAX_Ns))
        deriv = (err - self._prev_err) / self.dt
        self._prev_err = err
        if not self.ever_in_contact:
            return 0.0
        v = self.kp * err + self.ki * self._integral + self.kd * deriv
        return float(np.clip(v, -_PID_V_MAX_MS, _PID_V_MAX_MS))


class _StiffnessEstimator:
    """Estima a rigidez de contato K = ΔF/Δx (N/m) online, por EMA.

    Alimentado a cada tick com o Δx COMANDADO ao longo do approach (m) e a
    força medida (N). Usa-se o passo comandado, e não o TCP logado — este
    fica quantizado demais (~µm) para diferenciar de forma confiável. O
    erro de fase (a força responde com ~1 tick de atraso do JTC) é absorvido
    pelo filtro EMA e pela sub-relaxação do deadbeat. K é saturada em
    [_K_MIN_NM, _K_MAX_NM]; antes da 1ª amostra válida vale _K_DEFAULT_NM.
    """

    def __init__(self):
        self.reset()

    def reset(self, k0: float = _K_DEFAULT_NM):
        self.k = float(k0)
        self._f_prev: float | None = None
        self.estimated = False

    def update(self, dx_cmd_m: float, f_now: float, in_contact: bool):
        f_prev = self._f_prev
        self._f_prev = f_now
        # Limiar de Δx: abaixo disso k_inst = ΔF/Δx vira ruído puro. Era 20 µm,
        # mas os passos em CONTATO são de 15–18 µm (teto v_min×dt) — o
        # estimador nunca atualizava e K ficava preso no default de 40 N/mm
        # (todos os logs de 30/06 mostram K_est=40). Com 8 µm os passos de
        # contato alimentam o estimador; o EMA absorve o ruído extra.
        if not in_contact or f_prev is None or abs(dx_cmd_m) < 8e-6:
            return
        k_inst = (f_now - f_prev) / dx_cmd_m   # N/m (assinado: dx e dF mesmo sinal)
        if _K_MIN_NM <= k_inst <= _K_MAX_NM:
            a = _K_EMA_ALPHA if self.estimated else 1.0
            self.k = (1.0 - a) * self.k + a * k_inst
            self.estimated = True

    def update_pair(self, dx_m: float, df_n: float):
        """Par (Δx executado, ΔF medido) com AMBAS as forças lidas em REPOUSO
        (modo quase-estático) — sem o erro de fase do update() contínuo, o
        k_inst é a rigidez real do trecho percorrido. ΔF pequeno demais é
        descartado: seria creep/relaxação do contato, não elasticidade."""
        if abs(dx_m) < 1.5e-6 or abs(df_n) < 0.1:
            return
        k_inst = df_n / dx_m
        if _K_MIN_NM <= k_inst <= _K_MAX_NM:
            a = _K_EMA_ALPHA if self.estimated else 1.0
            self.k = (1.0 - a) * self.k + a * k_inst
            self.estimated = True

    @property
    def value(self) -> float:
        return float(min(max(self.k, _K_MIN_NM), _K_MAX_NM))


class TactileExplorer(Node):

    def __init__(self):
        super().__init__('tactile_explorer')

        self.declare_parameter('retract_mm',          80.0)
        self.declare_parameter('arm_base_z',          0.78)
        # Modo MovL: os movimentos da palpação são executados pelo ROBÔ REAL
        # via MovJ/RelMovL (intents JSON em /palpation/real_cmd, executados
        # pela GUI) em vez do streaming de trajetórias ao Gazebo. Só entra em
        # vigor quando a GUI reporta real conectado + MIRROR em
        # /palpation/real_movl; caso contrário o streaming clássico continua.
        self.declare_parameter('real_movl', True)
        self.declare_parameter('approach_v_max_mms',  50.0)
        self.declare_parameter('approach_v_min_mms',   5.0)
        self.declare_parameter('descent_speed_mms', 5.0)
        self.declare_parameter('home_speed_rad_s', _HOME_MAX_RAD_S)
        # Ganhos do PID de força. PI por padrão (kd=0): a derivada amplifica
        # o ruído da célula de carga; o próprio contato já amortece o loop.
        self.declare_parameter('kp', 0.001)    # (m/s)/N
        self.declare_parameter('ki', 0.0005)   # (m/s)/(N·s)
        self.declare_parameter('kd', 0.0)      # (m/s)/(N/s)

        self._phase: str = 'IDLE'
        self._busy = threading.Event()
        self._params_lock = threading.Lock()
        self._target_depth_mm: float = 5.0
        self._target_force_n:  float = 2.0   # setpoint do PID (≤ 10 N)
        self._kp = float(self.get_parameter('kp').value)
        self._ki = float(self.get_parameter('ki').value)
        self._kd = float(self.get_parameter('kd').value)
        # Rigidez de contato estimada durante o DESCENDING e reusada (congelada)
        # no HOLD/SLIDING para o passo deadbeat normalizado pela rigidez.
        self._k_est = _StiffnessEstimator()
        # Profundidade (m ao longo do approach) onde o último DESCENDING tocou.
        # Habilita a descida em dois estágios — ver _CONTACT_ZONE_MARGIN_M.
        self._learned_contact_m: float | None = None
        self._target_slide_mm: float = 50.0
        self._slide_speed_mms: float = 10.0
        # Modo do experimento: 'SLIDE' (deslizamento) ou 'TOUCH' (toque).
        # Em TOUCH pula-se a fase SLIDING — apenas descida com força
        # controlada (HOLD) e recuo, repetido `_repeats` vezes (toques).
        self._mode: str = 'SLIDE'
        self._slide_dir_vec: np.ndarray = np.array([0.0, 1.0])
        self._approach_dir: np.ndarray | None = None
        self._user_home_q: np.ndarray | None = None
        self._speed_factor_pct: float = 10.0   # % do slider da GUI (padrão 10 %)
        # Repetições automáticas do experimento (campo 'repeats' da GUI).
        # _cycle/_cycles_total alimentam o status para a GUI mostrar "i/N".
        self._repeats: int = 1
        self._cycle: int = 0
        self._cycles_total: int = 1
        # Overrides de estabilização do HOLD vindos do PalpationStart
        # (0.0 no msg = "usar default" → None aqui).
        self._hold_tol_n: float | None = None
        self._hold_stable_s: float | None = None
        self._hold_timeout_s: float | None = None
        self._lc_lock = threading.Lock()
        self._lc_force_net: float = 0.0   # compressão positiva, tare-compensada
        self._lc_force_ts: float = 0.0    # time.monotonic() da última leitura
        self._q_lock = threading.Lock()
        self._current_q = _POINTING_SEED_Q.copy()
        self._stop_requested = threading.Event()
        self._pause_requested = threading.Event()
        self._protocol_thread: threading.Thread | None = None
        # ─── Modo MovL (robô real executa; sim espelha o feedback) ────
        self._movl_param = bool(self.get_parameter('real_movl').value)
        self._movl_avail = False   # GUI: real conectado + MIRROR + movl on
        self._movl_run = False     # snapshot por experimento (_run_protocol)

        cb = ReentrantCallbackGroup()

        self.create_subscription(PalpationStart, '/palpation/start',
                                  self._cb_start, _QOS_COMMAND, callback_group=cb)
        self.create_subscription(String, '/palpation/stop',
                                  self._cb_stop, 10, callback_group=cb)
        self.create_subscription(Bool, '/palpation/pause',
                                  self._cb_pause, 10, callback_group=cb)
        self.create_subscription(Float32, '/load_cell/force_net',
                                  self._cb_lc_force_net, _QOS_SENSOR, callback_group=cb)
        self.create_subscription(JointState, '/joint_states',
                                  self._cb_joints, 50, callback_group=cb)

        self._status_pub = self.create_publisher(
            PalpationStatus, '/palpation/status', 10)

        # Canal do modo MovL: intents JSON para a GUI executar no robô real
        # (rel/movj/halt/calibrate_frame/run_begin/run_end) e disponibilidade
        # reportada pela GUI (Bool a ~1 Hz).
        self._real_cmd_pub = self.create_publisher(
            String, '/palpation/real_cmd', 10)
        self.create_subscription(Bool, '/palpation/real_movl',
                                 self._cb_movl_avail, 10, callback_group=cb)

        # Publisher direto no tópico do controller — sem action server.
        # depth=1: sem fila; cada nova mensagem substitui a anterior para
        # evitar rajada de setpoints antigos após jitter do SO.
        self._arm_traj_pub = self.create_publisher(
            JointTrajectory,
            '/cr10_group_controller/joint_trajectory', 1)
        self._hand_pub = self.create_publisher(
            JointTrajectory,
            '/hand_position_controller/joint_trajectory', 5)

        self.get_logger().info('tactile_explorer pronto — streaming 33 Hz')
        self.create_timer(0.10, self._publish_status, callback_group=cb)

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────
    _LC_MAX_PLAUSIBLE_N = 100.0

    def _cb_lc_force_net(self, msg: Float32) -> None:
        """Recebe /load_cell/force_net — compressão positiva, tare-compensada."""
        val = float(msg.data)
        if not math.isfinite(val) or abs(val) > self._LC_MAX_PLAUSIBLE_N:
            return
        with self._lc_lock:
            self._lc_force_net = val
            self._lc_force_ts = time.monotonic()

    def _cb_joints(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        with self._q_lock:
            for i, j in enumerate(_ARM_JOINTS):
                if j in idx:
                    self._current_q[i] = float(msg.position[idx[j]])

    def _cb_movl_avail(self, msg: Bool) -> None:
        """GUI reporta se o modo MovL pode ser usado (real conectado+MIRROR).
        Snapshot em _run_protocol — não muda no meio de um experimento."""
        self._movl_avail = bool(msg.data)

    def _cb_stop(self, msg: String) -> None:
        if self._busy.is_set():
            self._stop_requested.set()
            self._pause_requested.clear()   # stop vence pausa
            if self._movl_run:
                self._real_cmd('halt')   # aborta o segmento MovL em curso
            self.get_logger().warn('[STOP] Parada solicitada.')

    def _cb_pause(self, msg: Bool) -> None:
        """Pausa/retoma o experimento — as fases seguram a posição atual
        enquanto pausadas (ver _pause_gate)."""
        if bool(msg.data):
            if self._busy.is_set():
                self._pause_requested.set()
        else:
            self._pause_requested.clear()

    def _cb_start(self, msg: PalpationStart):
        if self._busy.is_set():
            self.get_logger().warn(
                f'Recebido /palpation/start mas explorer está em '
                f'{self._phase}. Ignorando.')
            return
        with self._params_lock:
            self._target_depth_mm = float(msg.depth_mm)
            # Setpoint do PID de força — saturado no máximo selecionável.
            self._target_force_n = float(np.clip(
                float(msg.force_n), 0.1, _FORCE_SETPOINT_MAX_N))
            self._kp = float(msg.kp)
            self._ki = float(msg.ki)
            self._kd = float(msg.kd)
            self._target_slide_mm = float(msg.slide_dist_mm)
            self._slide_speed_mms = float(msg.speed_mms)
            mode = str(msg.mode).upper().strip()
            self._mode = mode if mode in ('SLIDE', 'TOUCH') else 'SLIDE'
            if msg.approach_speed_mms > 0.0:
                v_max = max(1.0, float(msg.approach_speed_mms))
                v_min = max(0.5, v_max * 0.2)
                self.set_parameters([
                    rclpy.parameter.Parameter(
                        'approach_v_max_mms',
                        rclpy.parameter.Parameter.Type.DOUBLE, v_max),
                    rclpy.parameter.Parameter(
                        'approach_v_min_mms',
                        rclpy.parameter.Parameter.Type.DOUBLE, v_min),
                ])
            if msg.speed_factor_pct > 0.0:
                self._speed_factor_pct = float(
                    max(1.0, min(100.0, float(msg.speed_factor_pct))))
            self._repeats = int(np.clip(int(msg.repeats) or 1, 1, 100))
            slide_dir = str(msg.slide_dir).upper().strip() or '+Y'
            _DIR_MAP = {
                '+X': (1.0, 0.0), '-X': (-1.0, 0.0),
                '+Y': (0.0, 1.0), '-Y': (0.0, -1.0),
            }
            if slide_dir in _DIR_MAP:
                self._slide_dir_vec = np.array(_DIR_MAP[slide_dir])
            else:
                self.get_logger().warn(
                    f'slide_dir inválido "{slide_dir}" — usando +Y.')
                self._slide_dir_vec = np.array([0.0, 1.0])
            self._user_home_q = np.array(
                [math.radians(float(v)) for v in msg.home_deg],
                dtype=np.float64)
            # Estabilização do HOLD — 0.0 no msg = usar default do explorer.
            self._hold_tol_n = (float(msg.hold_tol_n)
                                if msg.hold_tol_n > 0.0 else None)
            self._hold_stable_s = (float(msg.hold_stable_s)
                                   if msg.hold_stable_s > 0.0 else None)
            self._hold_timeout_s = (float(msg.hold_timeout_s)
                                    if msg.hold_timeout_s > 0.0 else None)
        self._pause_requested.clear()
        self._protocol_thread = threading.Thread(
            target=self._run_protocol, daemon=True)
        self._protocol_thread.start()

    # ──────────────────────────────────────────────────────────────────
    # Status
    # ──────────────────────────────────────────────────────────────────
    def _publish_status(self):
        with self._lc_lock:
            force_net = self._lc_force_net
        with self._params_lock:
            depth_mm  = float(self._target_depth_mm)
            speed_mms = float(self._slide_speed_mms)
            target_f  = float(self._target_force_n)
        msg = PalpationStatus()
        msg.phase = self._phase
        msg.cycle = int(self._cycle)
        msg.cycles_total = int(self._cycles_total)
        msg.target_depth_mm = depth_mm
        msg.target_force_n = target_f
        msg.force_net_n = float(force_net)
        msg.speed_mms = speed_mms
        msg.paused = self._pause_requested.is_set()
        self._status_pub.publish(msg)

    def _fz_corrected(self) -> float:
        """Força de contato tare-compensada (N). Positivo = compressão."""
        with self._lc_lock:
            return self._lc_force_net

    def _force_stale_abort(self, phase: str) -> bool:
        """True se a leitura de força está velha/ausente — a fase chamadora
        deve abortar com outcome 'stale'. Loga o motivo uma única vez."""
        with self._lc_lock:
            ts = self._lc_force_ts
        if ts > 0.0:
            age = time.monotonic() - ts
            if age <= _FORCE_STALE_S:
                return False
            detail = f'última leitura há {age:.1f} s (> {_FORCE_STALE_S:.1f} s)'
        else:
            detail = 'nenhuma leitura recebida em /load_cell/force_net'
        self.get_logger().error(
            f'SEGURANÇA [{phase}]: célula de carga sem dados frescos — '
            f'{detail}. Controle por força não confiável; abortando. '
            'Verifique a ESP32 e o force_receiver.')
        return True

    def _pause_gate(self) -> bool:
        """Bloqueia enquanto o experimento estiver pausado, segurando a
        posição atual (re-publica o setpoint corrente como o _settle).
        Retorna False se um STOP chegar durante a pausa."""
        if not self._pause_requested.is_set():
            return True
        self.get_logger().warn('[PAUSE] experimento pausado — segurando posição.')
        if self._movl_run:
            # Aborta o segmento MovL em curso — sem isso o robô continuaria
            # viajando durante a pausa. O chamador re-emite o restante.
            self._real_cmd('halt')
        q_hold = self._q_now()
        zero_vel = np.zeros(6)
        while self._pause_requested.is_set():
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn('[PAUSE] stop durante a pausa.')
                return False
            self._stream_q(q_hold, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)
        self.get_logger().info('[PAUSE] experimento retomado.')
        return True

    def _set_phase(self, phase: str):
        self._phase = phase
        self.get_logger().info(f'[FSM] → {phase}')
        self._publish_status()

    # ──────────────────────────────────────────────────────────────────
    # Primitiva de streaming — 1 ponto por mensagem, substitui o goal
    # atual no controller (sem queue). Chamada a cada _CTRL_DT segundos.
    # ──────────────────────────────────────────────────────────────────
    def _stream_q(self, q: np.ndarray, dt_s: float,
                  velocities: np.ndarray | None = None) -> None:
        """Publica 1 setpoint. time_from_start = dt_s (lookahead do ctrl).

        Quando `velocities` é fornecido (rad/s por junta), o JTC usa splines
        cúbicos contínuos em velocidade — elimina a descontinuidade que
        acontecia ao encadear mensagens de 1 ponto sem hints de velocidade.
        """
        # Modo MovL: o sim é espelhado do feedback real pela GUI — qualquer
        # publicação daqui disputaria o JTC com o espelho. Holds/settles
        # tornam-se no-ops (o robô real segura posição sozinho).
        if self._movl_run:
            return
        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        if velocities is not None:
            pt.velocities = [float(v) for v in velocities]
        sec = int(dt_s)
        pt.time_from_start = Duration(sec=sec,
                                       nanosec=int((dt_s - sec) * 1e9))
        msg.points.append(pt)
        self._arm_traj_pub.publish(msg)

    def _real_cmd(self, op: str, **kw) -> None:
        """Publica um intent do modo MovL para a GUI executar no robô real.

        ops: 'rel' {d_mm:[dx,dy,dz] no MUNDO URDF} · 'movj' {q_urdf:[6] rad}
             'halt' · 'calibrate_frame' · 'run_begin' · 'run_end'
        """
        msg = String()
        msg.data = json.dumps({'op': op, **kw})
        self._real_cmd_pub.publish(msg)

    def _q_now(self) -> np.ndarray:
        with self._q_lock:
            return self._current_q.copy()

    # ──────────────────────────────────────────────────────────────────
    # _settle: publica posição atual por N ticks para zerar lookahead
    # e movimento residual antes de cada transição de fase.
    # ──────────────────────────────────────────────────────────────────
    def _settle(self, ticks: int = _SETTLE_TICKS) -> None:
        q = self._q_now()
        zero_vel = np.zeros(6)
        for _ in range(ticks):
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)

    def _settle_until_quiet(self, max_ticks: int = 20,
                            dfdt_tol_n: float = 0.05,
                            quiet_ticks: int = 3) -> None:
        """Trava a posição atual (velocidade zero) e ESPERA a força assentar
        antes de devolver o controle ao loop de força. Sai quando |ΔF| entre
        ticks fica < `dfdt_tol_n` por `quiet_ticks` consecutivos, ou ao atingir
        `max_ticks`. Absorve a inércia/lookahead herdados do DESCENDING contra
        contato rígido — é o que evita o pico no handoff DESCENDING→HOLD."""
        q = self._q_now()
        zero_vel = np.zeros(6)
        f_prev = self._fz_corrected()
        quiet = 0
        for _ in range(max_ticks):
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)
            f_now = self._fz_corrected()
            if abs(f_now - f_prev) < dfdt_tol_n:
                quiet += 1
                if quiet >= quiet_ticks:
                    return
            else:
                quiet = 0
            f_prev = f_now

    def _qs_measure_fz(self, q_hold: np.ndarray | None = None) -> float:
        """Mede a força em repouso (modo quase-estático): congela o braço por
        _QS_SETTLE_TICKS (o pipeline One-Euro + JTC esvazia) e devolve a
        mediana das últimas _QS_MEDIAN_N leituras.

        `q_hold` é a posição COMANDADA a segurar. Congelar na posição MEDIDA
        (q_now) cancelava qualquer micro-passo sub-LSB ainda não executado
        pelo drive — coletas de 04/07 14:51: passos "sem resposta" eram
        passos desfeitos pelo próprio congelamento do ciclo seguinte.

        Se a margem de segurança for cruzada durante a espera, devolve a
        leitura alta imediatamente — o chamador dispara o alívio."""
        q = self._q_now() if q_hold is None else q_hold
        zero_vel = np.zeros(6)
        reads: list[float] = []
        # Modo MovL: o RelMovL do micro-passo pode ainda estar na fila do
        # robô quando a medição começa (o dash retorna antes de executar) —
        # janela dobrada para medir com o passo realmente concluído.
        ticks = _QS_SETTLE_TICKS * (2 if self._movl_run else 1)
        for _ in range(ticks):
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)
            fz = self._fz_corrected()
            if fz > _FORCE_SAFE_LIMIT_N:
                return fz
            reads.append(fz)
        return float(np.median(reads[-_QS_MEDIAN_N:]))

    def _qs_step(self, approach_dir: np.ndarray, step_m: float,
                 v_lim: float, I6: np.ndarray,
                 q_from: np.ndarray | None = None) -> np.ndarray | None:
        """Executa UM micro-passo ao longo do approach em 1 tick, partindo da
        posição COMANDADA `q_from` (ou da medida, se None). Devolve o novo q
        comandado — o chamador congela NELE até o próximo passo."""
        q = self._q_now() if q_from is None else q_from
        if step_m == 0.0:
            time.sleep(_CTRL_DT)
            return q
        # Modo MovL: o micro-passo vira UM RelMovL real (linear por
        # construção); o congelamento em q_cmd não existe (o robô segura
        # posição sozinho), então devolve q inalterado como âncora.
        if self._movl_run:
            self._real_cmd('rel', d_mm=[float(v) for v in
                                        (approach_dir * step_m * 1e3)])
            time.sleep(_CTRL_DT)
            return q
        tw = np.zeros(6)
        tw[:3] = approach_dir * step_m
        J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
        try:
            dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, tw)
        except np.linalg.LinAlgError:
            return None
        q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
        vel = np.clip((q_new - q) / _CTRL_DT, -v_lim, v_lim)
        self._stream_q(q_new, _CTRL_DT, velocities=vel)
        time.sleep(_CTRL_DT)
        return q_new

    def _qs_regulate(self, target_f: float, tol_n: float,
                     approach_dir: np.ndarray, v_lim: float, I6: np.ndarray,
                     *, budget_m: float | None, stable_s: float,
                     timeout_s: float, phase: str) -> tuple[str, float]:
        """Regulação de força QUASE-ESTÁTICA (move-then-measure).

        Alterna medição em repouso (_qs_measure_fz) e UM micro-passo por
        ciclo, dimensionado para |ΔF projetado| ≤ _QS_DF_MAX_N. Dentro da
        banda não se move — a posição congelada É o hold; o relógio de
        estabilidade conta medições settled consecutivas em banda.

        Devolve (outcome, fz):
          'ok'      — banda mantida por stable_s contínuos
          'timeout' — não estabilizou em timeout_s
          'budget'  — precisaria aprofundar além de budget_m
          'force' | 'stale' | 'stop' — segurança / usuário
        """
        t_start = time.time()
        t_stable0: float | None = None
        deepened_m = 0.0
        fz_prev: float | None = None
        step_prev = 0.0
        boost = 1.0   # multiplicador anti-stiction (cresce se ΔF não responde)
        q_cmd: np.ndarray | None = None   # última posição COMANDADA — congela
                                          # nela, senão o freeze desfaz passos
                                          # sub-LSB ainda não executados
        while True:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn(f'[STOP] {phase} interrompido pelo usuário.')
                return 'stop', 0.0
            if not self._pause_gate():
                return 'stop', 0.0
            if self._force_stale_abort(phase):
                return 'stale', 0.0

            fz = self._qs_measure_fz(q_cmd)
            if fz > _FORCE_SAFE_LIMIT_N:
                self._relieve_contact(approach_dir)
                self.get_logger().error(
                    f'SEGURANÇA: compressão {fz:.1f} N > margem '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N (teto '
                    f'{_FORCE_ABORT_LIMIT_N:.0f} N) — medição cancelada.')
                return 'force', fz

            in_contact = fz > _CONTACT_ON_N
            if fz_prev is not None and step_prev != 0.0:
                if in_contact and fz_prev > _CONTACT_ON_N:
                    self._k_est.update_pair(step_prev, fz - fz_prev)
                boost = (min(boost * 1.5, _QS_BOOST_MAX)
                         if abs(fz - fz_prev) < _QS_DF_DEAD_N else 1.0)

            err = target_f - fz
            now = time.time()
            in_band = abs(err) <= tol_n
            if in_band:
                if t_stable0 is None:
                    t_stable0 = now
                    self.get_logger().info(
                        f'{phase}: em banda (fz={fz:.2f} N, alvo '
                        f'{target_f:.2f} ± {tol_n:.2f}) — segurando '
                        f'{stable_s:.1f} s.')
                elif now - t_stable0 >= stable_s:
                    return 'ok', fz
                if abs(err) <= 0.5 * tol_n:
                    # Centro da banda: congelado de verdade.
                    fz_prev, step_prev, boost = fz, 0.0, 1.0
                    continue
                # Meia-banda até a borda: micro-passo de perseguição do
                # creep SEM resetar a janela (banda é critério de força,
                # não de imobilidade) — sem isso o hold acampava na borda
                # inferior e a relaxação o arrastava para fora.
            else:
                if t_stable0 is not None:
                    self.get_logger().info(
                        f'{phase}: saiu da banda (fz={fz:.2f} N) — janela '
                        'de estabilidade reiniciada.')
                    t_stable0 = None
                if now - t_start >= timeout_s:
                    return 'timeout', fz

            k = self._k_est.value
            if not in_contact:
                step_m = min(_QS_FREE_STEP_M * boost, _QS_FREE_STEP_MAX_M)
            else:
                # boost multiplica o PASSO: se o passo calculado for menor
                # que a resolução real do atuador (stiction/LSB da junta),
                # sem resposta de ΔF ele cresce 1,5×/ciclo até o braço
                # efetivamente responder — mas o ΔF projetado tem teto DURO
                # (_QS_DF_HARD_N), boost incluso: sem ele o passo acumulado
                # estourava 7–9 N (coletas 04/07 14:51/14:52).
                step_m = _QS_RELAX * err * boost / k
                hard_cap = _QS_DF_HARD_N / k
                step_m = float(np.clip(step_m, -hard_cap, hard_cap))
                if not self._k_est.estimated:
                    # K desconhecida: o teto por ΔF projetado não protege —
                    # passo-sonda com teto ABSOLUTO, mesmo com boost.
                    probe_cap = min(_QS_DX_PROBE_M * boost, _QS_DX_PROBE_MAX_M)
                    step_m = float(np.clip(step_m, -probe_cap, probe_cap))
                if step_m < 0.0:
                    step_m = max(step_m, -(fz - _QS_RELIEF_FLOOR_N) / k)
            step_m = float(np.clip(step_m, -_QS_DX_MAX_M, _QS_DX_MAX_M))
            if budget_m is not None and deepened_m + step_m > budget_m:
                step_m = budget_m - deepened_m
                if step_m <= 1e-7 and err > 0.0:
                    return 'budget', fz
            q_new = self._qs_step(approach_dir, step_m, v_lim, I6,
                                  q_from=q_cmd)
            if q_new is not None:
                q_cmd = q_new
                deepened_m += step_m
                fz_prev, step_prev = fz, step_m

    def _relieve_contact(self, approach_dir: np.ndarray,
                         max_ticks: int = 20) -> None:
        """Alívio de EMERGÊNCIA ao cruzar a margem de força: recua ao longo
        do approach a passo cheio (0.25 mm/tick ≈ 8 mm/s) até a compressão
        cair abaixo de metade da margem, então trava a posição. Congelar no
        lugar (_settle) MANTINHA a compressão — na coleta de 02/07 a força
        ficou 90 ms acima do teto de 15 N esperando o abort subir à home."""
        if self._movl_run:
            # Aborta o segmento em curso e recua em passos lineares de 0.5 mm
            # até a compressão cair abaixo de metade da margem (teto 5 mm,
            # mesmo curso total do caminho streaming).
            self._real_cmd('halt')
            back = (-np.asarray(approach_dir, float)) * 0.5   # mm
            for _ in range(10):
                if self._fz_corrected() < 0.5 * _FORCE_SAFE_LIMIT_N:
                    break
                self._real_cmd('rel', d_mm=[float(v) for v in back])
                time.sleep(0.15)
            self._settle()
            return
        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        for _ in range(max_ticks):
            if self._fz_corrected() < 0.5 * _FORCE_SAFE_LIMIT_N:
                break
            tw = np.zeros(6)
            tw[:3] = -approach_dir * _DEADBEAT_DX_MAX_M
            q = self._q_now()
            J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, tw)
            except np.linalg.LinAlgError:
                break
            q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
            vel = np.clip((q_new - q) / _CTRL_DT, -v_lim, v_lim)
            self._stream_q(q_new, _CTRL_DT, velocities=vel)
            time.sleep(_CTRL_DT)
        self._settle()

    def _home_v_rad_s(self) -> float:
        """Velocidade máxima por junta dos retornos HOME (rad/s), saturada."""
        try:
            v = float(self.get_parameter('home_speed_rad_s').value)
        except Exception:
            v = _HOME_MAX_RAD_S
        return float(min(max(v, 0.01), 0.30))

    # ──────────────────────────────────────────────────────────────────
    # Movimento no espaço de juntas: interpola linearmente q_from → q_to
    # a velocidade máxima de home_speed_rad_s rad/s por junta.
    # ──────────────────────────────────────────────────────────────────
    def _movl_joint_to(self, q_target: np.ndarray,
                       timeout_s: float = 60.0) -> bool:
        """Modo MovL: envia UM MovJ articular ao robô real e espera o
        feedback espelhado chegar ao alvo. Substitui os streams articulares
        (home/goto) — o perfil de velocidade é o do próprio robô."""
        q_target = np.asarray(q_target, float)
        if float(np.max(np.abs(q_target - self._q_now()))) < 0.001:
            return True
        self._real_cmd('movj', q_urdf=[float(v) for v in q_target])
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self._real_cmd('halt')
                return False
            if not self._pause_gate():
                return False
            if float(np.max(np.abs(self._q_now() - q_target))) < 0.02:
                return True
            time.sleep(_CTRL_DT)
        self.get_logger().warn(
            f'[MOVL] movj não alcançou o alvo em {timeout_s:.0f} s '
            f'(Δmax={math.degrees(float(np.max(np.abs(self._q_now() - q_target)))):.2f}°) '
            '— confira conexão/velocidade do robô real.')
        return False

    def _joint_stream_to(self, q_target: np.ndarray) -> bool:
        if self._movl_run:
            return self._movl_joint_to(q_target)
        q_from = self._q_now()
        delta = np.asarray(q_target, float) - q_from
        max_d = float(np.max(np.abs(delta)))
        if max_d < 0.001:
            return True
        n_steps = max(1, int(math.ceil(max_d / (self._home_v_rad_s() * _CTRL_DT))))
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        vel_peak = np.clip(delta / n_steps / _CTRL_DT, -v_lim, v_lim)
        # Rampa trapezoidal: ~20 % de aceleração/desaceleração (máx 8 passos = 240 ms).
        # Evita o solavanco de arranque causado por velocidade constante desde t=0.
        ramp = min(max(1, n_steps // 5), 8)
        for i in range(1, n_steps + 1):
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                return False
            alpha = i / n_steps
            q = np.clip(q_from + alpha * delta, JOINT_MIN, JOINT_MAX)
            if i <= ramp:
                scale = i / ramp
            elif i >= n_steps - ramp + 1:
                scale = (n_steps - i + 1) / ramp
            else:
                scale = 1.0
            step_vel = vel_peak * scale if i < n_steps else np.zeros(6)
            self._stream_q(q, _CTRL_DT, velocities=step_vel)
            time.sleep(_CTRL_DT)
        return True

    # ──────────────────────────────────────────────────────────────────
    # Movimento Cartesiano retilíneo por streaming Jacobiano a 33 Hz.
    #
    # Cada iteração:
    #   1. Verifica stop / força
    #   2. Calcula step_m a partir do perfil de velocidade
    #   3. Aplica twist Jacobiano DLS (translação + correção de orientação)
    #   4. Publica 1 setpoint via _stream_q
    #   5. Dorme _CTRL_DT
    #
    # Sem pré-planejamento: o próximo passo é calculado depois do anterior.
    # Não há fila — cada mensagem substitui a anterior no controller.
    #
    # Retorna: 'done' | 'force' | 'stop' | 'error'
    # ──────────────────────────────────────────────────────────────────
    def _cartesian_stream(self, direction: np.ndarray, total_m: float, *,
                           v_const_ms: float | None = None,
                           v_max_ms: float | None = None,
                           v_min_ms: float | None = None,
                           lock_ori: bool = False,
                           lock_z: bool = False,
                           lock_perp: bool = False,
                           force_threshold_n: float | None = None,
                           win: int = _CTRL_WIN) -> str:
        d = np.asarray(direction, dtype=float).flatten()
        nd = float(np.linalg.norm(d))
        if nd < 1e-9 or total_m <= 0.0:
            self.get_logger().error('_cartesian_stream: direção/distância inválida.')
            return 'error'
        d /= nd

        constant = v_const_ms is not None
        if not constant and (v_max_ms is None or v_min_ms is None):
            self.get_logger().error(
                '_cartesian_stream: forneça v_const_ms OU (v_max_ms, v_min_ms).')
            return 'error'

        I6 = np.eye(6)

        # FK inicial — sempre calculada, independente de lock_*.
        # p_start é a âncora para medir o progresso real do TCP via FK,
        # substituindo a integração de passos comandados (que acumula erro
        # por aproximação do Jacobiano e clipping de limites articulares).
        T0 = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        p_start = T0[:3, 3].copy()

        R0: np.ndarray | None = None
        z0: float | None = None
        perp_dir: np.ndarray | None = None
        p0_perp: float | None = None
        if lock_ori:
            R0 = T0[:3, :3].copy()
        if lock_z:
            z0 = float(T0[2, 3])
        if lock_perp:
            # Perpendicular a d no plano XY. Para d = [0,0,±1] a norma é zero
            # e a correção é suprimida automaticamente.
            perp = np.array([-d[1], d[0], 0.0])
            pnorm = float(np.linalg.norm(perp))
            if pnorm > 1e-9:
                perp_dir = perp / pnorm
                p0_perp = float(p_start @ perp_dir)

        # Safety: timeout baseado em 10× o tempo nominal + margem de 30 s.
        v_est = float(v_const_ms) if constant else float(v_max_ms)
        v_est = max(1e-4, v_est)
        _timeout_s = max(30.0, (total_m / v_est) * 10.0)
        _t0 = time.time()
        # Detecção de direção errada: > 5 mm na direção negativa por > 3 s.
        _neg_ticks = 0
        _NEG_MAX = int(3.0 / _CTRL_DT)
        # Log diagnóstico a cada 1 s.
        _log_every = max(1, int(1.0 / _CTRL_DT))
        _tick = 0

        self.get_logger().info(
            f'_cartesian_stream: d={d.round(3)} total={total_m*1e3:.1f}mm '
            f'v={v_est*1e3:.1f}mm/s p_start={p_start.round(4)} '
            f'TCP_Z={T0[:3,2].round(3)}')

        # Progresso real do TCP na direção d (metros, medido via FK a cada tick).
        progress = 0.0

        while progress < total_m:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                return 'stop'

            # Timeout global — evita loop eterno se o robô não se move.
            if time.time() - _t0 > _timeout_s:
                self.get_logger().error(
                    f'_cartesian_stream: timeout {_timeout_s:.0f}s '
                    f'(progress={progress*1e3:.1f}mm/{total_m*1e3:.1f}mm). Abortando.')
                return 'error'

            if force_threshold_n is not None:
                if self._fz_corrected() >= force_threshold_n:
                    return 'force'

            q = self._q_now().copy()

            # FK do tick atual — única chamada por iteração.
            # Serve tanto para medir o progresso real quanto para as correções
            # de orientação, Z e perpendicular.
            T_cur = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)
            progress = float(np.dot(T_cur[:3, 3] - p_start, d))

            # Detecção de direção errada: se TCP persistentemente se afasta
            # de p_start na direção oposta a d, o Jacobiano provavelmente está
            # sendo calculado numa configuração errada. Aborta para não bloquear.
            if progress < -0.005:
                _neg_ticks += 1
                if _neg_ticks > _NEG_MAX:
                    self.get_logger().error(
                        f'_cartesian_stream: TCP na direção errada '
                        f'(progress={progress*1e3:.1f}mm por >{3.0:.0f}s). '
                        f'TCP_cur={T_cur[:3,3].round(4)} p_start={p_start.round(4)}. '
                        'Abortando.')
                    return 'error'
            else:
                _neg_ticks = 0

            # Log periódico para diagnóstico.
            if _tick % _log_every == 0:
                self.get_logger().debug(
                    f'  t={_tick*_CTRL_DT:.1f}s progress={progress*1e3:.2f}mm '
                    f'TCP={T_cur[:3,3].round(4)}')
            _tick += 1

            # Perfil de velocidade usa o progresso FK (não passos acumulados).
            u = max(0.0, min(1.0, progress / total_m))
            if constant:
                v = float(v_const_ms)
            else:
                v = float(v_min_ms) + (float(v_max_ms) - float(v_min_ms)) * (1.0 - u) ** 2
            v = max(1e-4, v)
            step = v * _CTRL_DT

            # ── Batch de `win` waypoints (janela deslizante) ─────────────────
            # Cada mensagem contém `win` pontos com timestamps cumulativos.
            # O JTC interpola S-curve sobre toda a janela → coordenação suave
            # de múltiplas juntas sem reinício de planejamento entre passos.
            # A janela desliza 1 passo (_CTRL_DT) por tick; a cada 30 ms
            # o batch é atualizado com o estado real lido de /joint_states.
            msg = JointTrajectory()
            msg.joint_names = list(_ARM_JOINTS)
            q_iter = q.copy()
            T_iter = T_cur
            v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
            singular = False
            for k in range(1, win + 1):
                tw = np.zeros(6)
                tw[:3] = d * step
                if R0 is not None:
                    R_err = R0 @ T_iter[:3, :3].T
                    tw[3:] = _ORI_GAIN * 0.5 * np.array([
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ])
                if z0 is not None:
                    tw[2] += _Z_CORR_GAIN * (z0 - float(T_iter[2, 3]))
                if perp_dir is not None and p0_perp is not None:
                    perp_err = p0_perp - float(T_iter[:3, 3] @ perp_dir)
                    tw[:3] += _Z_CORR_GAIN * perp_err * perp_dir
                J_k = jacobian(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
                try:
                    dq_k = J_k.T @ np.linalg.solve(
                        J_k @ J_k.T + _JAC_LAM**2 * I6, tw)
                except np.linalg.LinAlgError:
                    singular = True
                    break
                q_next = np.clip(q_iter + dq_k, JOINT_MIN, JOINT_MAX)
                vel_k = np.clip((q_next - q_iter) / _CTRL_DT, -v_lim, v_lim)
                pt = JointTrajectoryPoint()
                pt.positions = [float(x) for x in q_next]
                pt.velocities = [float(x) for x in vel_k]
                t_k = k * _CTRL_DT
                pt.time_from_start = Duration(
                    sec=int(t_k), nanosec=int((t_k - int(t_k)) * 1e9))
                msg.points.append(pt)
                q_iter = q_next
                if k < win:
                    T_iter = forward_kinematics(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
            if singular:
                self.get_logger().warn('Jacobiano singular — passo descartado.')
                time.sleep(_CTRL_DT)
                continue
            if msg.points:
                self._arm_traj_pub.publish(msg)
            time.sleep(_CTRL_DT)

        return 'done'

    # ──────────────────────────────────────────────────────────────────
    # Trajetória Cartesiana em batch completo (SLIDING).
    #
    # Pré-computa todos os N waypoints via Jacobiano iterado e os envia
    # em UMA única JointTrajectory. O JTC planeja a S-curve sobre o
    # conjunto inteiro — sem reinício a cada tick, sem instabilidade de
    # coordenação multi-junta independente da distância percorrida.
    #
    # Não monitora força: use _cartesian_stream para fases reativas.
    # Retorna: 'done' | 'stop' | 'error'
    # ──────────────────────────────────────────────────────────────────
    def _cartesian_batch_to(self, direction: np.ndarray, total_m: float, *,
                              v_const_ms: float | None = None,
                              v_max_ms: float | None = None,
                              v_min_ms: float | None = None,
                              lock_ori: bool = False,
                              lock_z: bool = False,
                              lock_perp: bool = False) -> str:
        d = np.asarray(direction, dtype=float).flatten()
        nd = float(np.linalg.norm(d))
        if nd < 1e-9 or total_m <= 0.0:
            self.get_logger().error('_cartesian_batch_to: direção/distância inválida.')
            return 'error'
        d /= nd

        constant = v_const_ms is not None
        if not constant and (v_max_ms is None or v_min_ms is None):
            self.get_logger().error(
                '_cartesian_batch_to: forneça v_const_ms OU (v_max_ms, v_min_ms).')
            return 'error'

        v_ref = float(v_const_ms) if constant else float(v_max_ms)
        v_ref = max(1e-4, v_ref)
        N = max(1, int(math.ceil(total_m / (v_ref * _CTRL_DT))))

        q = self._q_now()
        T0 = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)

        R0 = T0[:3, :3].copy() if lock_ori else None
        z0 = float(T0[2, 3]) if lock_z else None
        perp_dir: np.ndarray | None = None
        p0_perp: float | None = None
        if lock_perp:
            perp = np.array([-d[1], d[0], 0.0])
            pnorm = float(np.linalg.norm(perp))
            if pnorm > 1e-9:
                perp_dir = perp / pnorm
                p0_perp = float(T0[:3, 3] @ perp_dir)

        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        self.get_logger().info(
            f'_cartesian_batch_to: pré-computando {N} waypoints '
            f'({total_m*1e3:.1f}mm @ {v_ref*1e3:.1f}mm/s) ...')

        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        q_iter = q.copy()
        T_iter = T0

        for k in range(1, N + 1):
            u = (k - 1) / max(1, N - 1)
            if constant:
                v_k = float(v_const_ms)
            else:
                v_k = float(v_min_ms) + (float(v_max_ms) - float(v_min_ms)) * (1.0 - u) ** 2
            v_k = max(1e-4, v_k)
            step = v_k * _CTRL_DT

            tw = np.zeros(6)
            tw[:3] = d * step
            if R0 is not None:
                R_err = R0 @ T_iter[:3, :3].T
                tw[3:] = _ORI_GAIN * 0.5 * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ])
            if z0 is not None:
                tw[2] += _Z_CORR_GAIN * (z0 - float(T_iter[2, 3]))
            if perp_dir is not None and p0_perp is not None:
                perp_err = p0_perp - float(T_iter[:3, 3] @ perp_dir)
                tw[:3] += _Z_CORR_GAIN * perp_err * perp_dir

            J_k = jacobian(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
            try:
                dq_k = J_k.T @ np.linalg.solve(J_k @ J_k.T + _JAC_LAM**2 * I6, tw)
            except np.linalg.LinAlgError:
                self.get_logger().warn(f'Batch: Jacobiano singular no passo {k} — truncando.')
                break

            q_next = np.clip(q_iter + dq_k, JOINT_MIN, JOINT_MAX)
            vel_k = np.clip((q_next - q_iter) / _CTRL_DT, -v_lim, v_lim)
            if k == N:
                vel_k = np.zeros(6)

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q_next]
            pt.velocities = [float(x) for x in vel_k]
            t_k = k * _CTRL_DT
            pt.time_from_start = Duration(
                sec=int(t_k), nanosec=int((t_k - int(t_k)) * 1e9))
            msg.points.append(pt)
            q_iter = q_next
            T_iter = forward_kinematics(q_iter, T_end=T_TOUCH_TOOL_ATTACH)

        if not msg.points:
            return 'error'

        self._arm_traj_pub.publish(msg)
        self.get_logger().info(
            f'_cartesian_batch_to: {len(msg.points)} pts publicados '
            f'(duração {len(msg.points)*_CTRL_DT:.1f}s)')

        t_end = time.monotonic() + len(msg.points) * _CTRL_DT + 0.5
        while time.monotonic() < t_end:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self._settle()
                return 'stop'
            time.sleep(_CTRL_DT)
        return 'done'

    # ──────────────────────────────────────────────────────────────────
    # Mão COVVI
    # ──────────────────────────────────────────────────────────────────
    def _send_hand_pose(self, primary_rad: dict[str, float],
                         duration_s: float | None = None) -> None:
        if duration_s is None:
            # Escala inversa ao speed_factor_pct: 10 % → 2.0 s, 100 % → 0.2 s
            duration_s = max(0.3, 2.0 * (10.0 / max(1.0, self._speed_factor_pct)))
        names = list(_HAND_PRIMARY)
        positions = [float(primary_rad.get(j, 0.0)) for j in _HAND_PRIMARY]
        for mimic_name, driver, mult in _MIMIC_LIST:
            names.append(mimic_name)
            positions.append(float(primary_rad.get(driver, 0.0)) * mult)
        msg = JointTrajectory()
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        dur = max(0.1, float(duration_s))
        pt.time_from_start = Duration(
            sec=int(dur), nanosec=int((dur - int(dur)) * 1e9))
        msg.points.append(pt)
        self._hand_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    # HOME: trajectória batch (uma mensagem multi-ponto → JTC planeia S-curve)
    # ──────────────────────────────────────────────────────────────────
    def _joint_batch_to(self, q_target: np.ndarray) -> bool:
        """Envia uma única JointTrajectory com todos os waypoints ao JTC.

        Em vez de streaming de N goals individuais a 33 Hz (que o JTC trata
        como N trajectórias independentes), envia-os todos numa mensagem só.
        O JTC usa interpolação cúbica sobre o conjunto completo → curva de
        velocidade suave sem solavancos de arranque.

        Fallback para _joint_stream_to se a trajectória tiver < 2 pontos.
        """
        if self._movl_run:
            return self._movl_joint_to(q_target)
        q_from = self._q_now()
        delta = np.asarray(q_target, float) - q_from
        max_d = float(np.max(np.abs(delta)))
        if max_d < 0.001:
            return True
        n_steps = max(2, int(math.ceil(max_d / (self._home_v_rad_s() * _CTRL_DT))))
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        vel_peak = np.clip(delta / n_steps / _CTRL_DT, -v_lim, v_lim)
        ramp = min(max(1, n_steps // 5), 8)

        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        for i in range(1, n_steps + 1):
            alpha = i / n_steps
            q = np.clip(q_from + alpha * delta, JOINT_MIN, JOINT_MAX)
            if i <= ramp:
                scale = i / ramp
            elif i >= n_steps - ramp + 1:
                scale = (n_steps - i + 1) / ramp
            else:
                scale = 1.0
            step_vel = vel_peak * scale if i < n_steps else np.zeros(6)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [float(v) for v in step_vel]
            t_s = i * _CTRL_DT
            pt.time_from_start = Duration(sec=int(t_s),
                                          nanosec=int((t_s - int(t_s)) * 1e9))
            msg.points.append(pt)

        self._arm_traj_pub.publish(msg)

        # Aguardar a execução, monitorizando stop a cada tick.
        t_end = time.monotonic() + n_steps * _CTRL_DT + 0.3
        while time.monotonic() < t_end:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self._settle()
                return False
            time.sleep(_CTRL_DT)
        return True

    # ──────────────────────────────────────────────────────────────────
    # Fases
    # ──────────────────────────────────────────────────────────────────
    def _phase_goto_home(self) -> bool:
        """HOME — trajectória batch ao JTC (S-curve interna) a ≤ 0.3 rad/s."""
        self._set_phase('HOME')
        self._send_hand_pose(_HAND_POINTING_RAD)

        q_home = (self._user_home_q.copy()
                  if self._user_home_q is not None
                  else _POINTING_SEED_Q.copy())

        # Settle antes de mover — garante que não há lookahead residual.
        self._settle()

        if not self._joint_batch_to(q_home):
            return False

        # Modo MovL: calibra o mapeamento de frame mundo→DOBOT em ar livre
        # (a GUI faz 2 sondas laterais de ±5 mm e mede o delta via FK).
        # Idempotente — a GUI ignora se já calibrado nesta sessão.
        if self._movl_run:
            self._real_cmd('calibrate_frame')
            time.sleep(0.3)   # sondas correm na fila serial da GUI antes
                              # de qualquer rel subsequente

        # Settle final para estabilizar antes do CONTACT.
        self._settle(ticks=_SETTLE_TICKS * 3)

        # Verificação de orientação: o TCP deve estar apontando para baixo
        # (componente -Z da terceira coluna de R deve ser ≤ −0.7).
        # Se a home customizada tiver o TCP em orientação incorreta, a fase
        # CONTACT desceria na direção errada (lock_ori manteria a orientação ruim).
        q_actual = self._q_now()
        R_tcp = forward_kinematics(q_actual, T_end=T_TOUCH_TOOL_ATTACH)[:3, :3]
        tcp_z_world = R_tcp[:, 2]   # terceira coluna = eixo Z do TCP no frame mundo
        if tcp_z_world[2] > -0.5:
            self.get_logger().warn(
                f'HOME: TCP não está apontando para baixo '
                f'(tcp_z_world[2]={tcp_z_world[2]:.2f}, esperado < −0.5). '
                'Palpação continua mas a descida pode ser incorreta.')
        else:
            self.get_logger().info(
                f'HOME: orientação OK — tcp_z_world[2]={tcp_z_world[2]:.2f}')

        # NÃO sobrescrever _current_q: _cb_joints já mantém o valor correto
        # a partir de /joint_states. Forçar q_home aqui descartaria o estado
        # real e poderia causar salto no primeiro passo do CONTACT.
        return True

    def _phase_descending(self) -> str:
        """DESCENDING — desce ao longo do approach até a força atingir o setpoint.

        Controle por força: termina quando a compressão alcança o setpoint
        do PID (force_n da GUI, ≤ 10 N). A profundidade da GUI é o curso
        máximo de segurança.

        Retorna: 'ok' (setpoint atingido) | 'no_contact' (curso esgotado)
                 | 'force' (> 15 N) | 'stale' (célula sem dados frescos)
                 | 'stop' (usuário).
        """
        self._set_phase('DESCENDING')
        self._settle()

        # Calcula approach_dir a partir da pose atual do TCP (coluna Z do frame).
        T_pre = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        approach_dir = T_pre[:3, 2].copy()
        if float(np.linalg.norm(approach_dir)) < 0.1:
            approach_dir = np.array([0.0, 0.0, -1.0])
        self._approach_dir = approach_dir.copy()

        with self._params_lock:
            depth_m      = float(self._target_depth_mm) / 1000.0
            target_f     = float(self._target_force_n)
            approach_mms = float(self.get_parameter('approach_v_max_mms').value)
            sf = self._speed_factor_pct / 100.0
            # Velocidade de descida = approach_v_max × speed_factor (igual ao robô em 10 %)
            v_fast_ms = max(0.001, approach_mms * sf / 1000.0)
            learned_m = self._learned_contact_m

        # Perfil de velocidade em ar livre (dois estágios):
        #   COM profundidade aprendida — v_fast (GUI) até a margem antes do
        #   ponto de contato conhecido, depois rastejo (_DESCEND_TOUCH_V_MS).
        #   SEM ela (1ª descida da sessão) — o contato pode vir a qualquer
        #   momento: toda a descida respeita o teto de contato. O 1º tick após
        #   tocar penetra v_app·dt antes do loop reagir, gerando transiente
        #   ≈ v_app·lat·K — a velocidade no toque é o que limita esse pico.
        v_slow_ms      = min(v_fast_ms, _DESCEND_TOUCH_V_MS)
        v_unlearned_ms = min(v_fast_ms, _DESCEND_CONTACT_V_MAX_MS)

        # Banda de chegada: termina o DESCENDING já DENTRO da tolerância do
        # HOLD (não em fz>=alvo, que garante overshoot). Assim o handoff entra
        # com força ~no setpoint e velocidade ~zero. Usa a MESMA tolerância
        # que o HOLD vai usar (inclusive o override da GUI) — bandas
        # diferentes fariam o HOLD partir já fora da própria banda.
        with self._params_lock:
            tol_override = self._hold_tol_n
        exit_tol = (tol_override if tol_override is not None
                    else max(_HOLD_TOL_N, _HOLD_TOL_PCT * target_f))

        if depth_m <= 0.0:
            self.get_logger().warn('DESCENDING: profundidade = 0 mm — pulando fase.')
            return 'ok'
        I6    = np.eye(6)
        dt    = _CTRL_DT
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S

        if self._movl_run:
            self._k_est.reset()
            return self._movl_descend(approach_dir, depth_m, target_f,
                                      exit_tol, v_lim, I6)

        # Estimador de rigidez começa do default a cada toque (o objeto pode
        # mudar entre ciclos). É preenchido durante o contato e congelado para
        # o HOLD/SLIDING usarem o mesmo K.
        self._k_est.reset()

        descended_m = 0.0
        # Posição COMANDADA acumulada da descida em ar livre. Partir da
        # posição MEDIDA a cada tick descartava o resíduo sub-LSB do passo
        # anterior: a 0,5 mm/s o passo de 15 µm virava ~10 µm executados
        # (coletas de 04/07 15:21–15:23 desceram a 0,33 mm/s = 2/3 do
        # comandado). Ressincroniza com a medida se divergir (pausa/JTC).
        q_cmd_free: np.ndarray | None = None
        if learned_m is not None:
            zona = (f'RÁPIDA a {v_fast_ms*1e3:.1f} mm/s até '
                    f'{(learned_m - _CONTACT_ZONE_MARGIN_M)*1e3:.1f} mm '
                    f'(contato aprendido em {learned_m*1e3:.1f} mm), '
                    f'depois rastejo a {v_slow_ms*1e3:.2f} mm/s')
        else:
            zona = (f'{v_unlearned_ms*1e3:.1f} mm/s até contato '
                    f'(1ª descida — sem profundidade aprendida)')
        self.get_logger().info(
            f'DESCENDING: alvo={target_f:.2f} ± {exit_tol:.2f} N  '
            f'curso máx={depth_m * 1000:.1f} mm  aproximação {zona}, '
            f'em contato QUASE-ESTÁTICO (K0={_K_DEFAULT_NM/1000:.0f} N/mm)  '
            f'(approach={approach_mms:.0f} mm/s × {self._speed_factor_pct:.0f}%)')

        while descended_m < depth_m:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn('[STOP] DESCENDING interrompido pelo usuário.')
                return 'stop'
            if not self._pause_gate():
                return 'stop'

            t0 = time.time()

            if self._force_stale_abort('DESCENDING'):
                return 'stale'
            fz = self._fz_corrected()  # + compressão, − tração
            if fz > _FORCE_SAFE_LIMIT_N:
                self._relieve_contact(approach_dir)   # recua NA HORA
                self.get_logger().error(
                    f'SEGURANÇA: compressão {fz:.1f} N > margem '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N (teto {_FORCE_ABORT_LIMIT_N:.0f} N) '
                    f'— medição cancelada.')
                return 'force'

            if fz > _CONTACT_ON_N:
                # CONTATO: memoriza a profundidade para as próximas descidas
                # (dois estágios; segue drifts lentos da montagem) e entrega
                # à regulação QUASE-ESTÁTICA — a malha contínua contra
                # contato rígido virava quique (coletas de 04/07).
                if descended_m > 0.001:
                    with self._params_lock:
                        self._learned_contact_m = descended_m
                self.get_logger().info(
                    f'DESCENDING: contato em {descended_m * 1000:.1f} mm '
                    f'(fz={fz:.2f} N) — engatando regulação quase-estática.')
                out, fz_end = self._qs_regulate(
                    target_f, exit_tol, approach_dir, v_lim, I6,
                    budget_m=depth_m - descended_m,
                    stable_s=_QS_ARRIVE_S, timeout_s=_QS_TIMEOUT_S,
                    phase='DESCENDING-QS')
                if out == 'ok':
                    self.get_logger().info(
                        f'DESCENDING: alvo atingido — fz={fz_end:.2f} N '
                        f'(alvo {target_f:.2f} ± {exit_tol:.2f} N, '
                        f'{_QS_ARRIVE_S:.2f} s settled em banda)  '
                        f'K_est={self._k_est.value/1000:.0f} N/mm.')
                    return 'ok'
                if out == 'timeout':
                    self.get_logger().warn(
                        f'DESCENDING-QS: sem estabilizar em '
                        f'{_QS_TIMEOUT_S:.0f} s (fz={fz_end:.2f} N) — '
                        'entregando ao HOLD, que continua regulando.')
                    return 'ok'
                if out == 'budget':
                    self.get_logger().warn(
                        f'DESCENDING-QS: curso máximo esgotado sem sustentar '
                        f'{target_f:.2f} N (fz={fz_end:.2f} N).')
                    return 'no_contact'
                return out   # 'force' | 'stale' | 'stop'

            # Ar livre — dois estágios (ver perfil acima do loop).
            if learned_m is None:
                v_free = v_unlearned_ms
            elif descended_m < learned_m - _CONTACT_ZONE_MARGIN_M:
                v_free = v_fast_ms
            else:
                v_free = v_slow_ms
            step_m = min(v_free * dt, depth_m - descended_m)

            tw = np.zeros(6)
            tw[:3] = approach_dir * step_m

            q_meas = self._q_now()
            q = q_meas if q_cmd_free is None else q_cmd_free
            # Divergência comandado×medido (pausa, JTC atrasado): resync.
            if q_cmd_free is not None and \
                    float(np.max(np.abs(q_cmd_free - q_meas))) > 0.02:
                q = q_meas
            J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, tw)
            except np.linalg.LinAlgError:
                time.sleep(dt)
                continue

            q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
            vel   = np.clip((q_new - q) / dt, -v_lim, v_lim)
            self._stream_q(q_new, dt, velocities=vel)
            q_cmd_free = q_new
            descended_m += step_m

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        self.get_logger().warn(
            f'DESCENDING: curso máximo de {descended_m * 1000:.1f} mm esgotado '
            f'sem atingir {target_f:.2f} N (fz={self._fz_corrected():.2f} N) — '
            'abortando com retorno lento à home.')
        return 'no_contact'

    # ── Fases em modo MovL (robô real executa; monitores aqui) ──────────
    _MOVL_SEG_M = 0.002   # 2 mm por sub-segmento da descida: teto de curso
                          # runaway entre checagens de força, e a fila do
                          # robô nunca acumula mais que ~2 segmentos

    def _movl_descend(self, approach_dir: np.ndarray, depth_m: float,
                      target_f: float, exit_tol: float,
                      v_lim: float, I6: np.ndarray) -> str:
        """DESCENDING em modo MovL: sub-segmentos RelMovL reais com monitor
        de força a 33 Hz; no contato, halt + regulação quase-estática (a
        mesma _qs_regulate — os micro-passos dela já saem como RelMovL).

        A velocidade linear real segue o SpeedFactor global da GUI (não há
        mapeamento mm/s→% validado); os sub-segmentos de 2 mm limitam o
        curso máximo entre checagens de força.
        """
        p_start = forward_kinematics(
            self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3].copy()
        d = np.asarray(approach_dir, float)
        sent_m = 0.0
        progress = 0.0
        t_deadline = time.time() + 120.0
        self.get_logger().info(
            f'[MOVL] DESCENDING: curso máx {depth_m*1e3:.1f} mm em '
            f'sub-segmentos de {self._MOVL_SEG_M*1e3:.1f} mm.')

        while True:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self._real_cmd('halt')
                self.get_logger().warn('[STOP] DESCENDING interrompido pelo usuário.')
                return 'stop'
            was_paused = self._pause_requested.is_set()
            if not self._pause_gate():
                return 'stop'
            if was_paused:
                # O halt da pausa descartou o que estava na fila —
                # ressincroniza o enviado com o executado e re-emite.
                progress = float(np.dot(forward_kinematics(
                    self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
                    - p_start, d))
                sent_m = max(0.0, progress)

            if self._force_stale_abort('DESCENDING'):
                self._real_cmd('halt')
                return 'stale'
            fz = self._fz_corrected()
            if fz > _FORCE_SAFE_LIMIT_N:
                self._relieve_contact(d)   # movl: halt + recuo linear
                self.get_logger().error(
                    f'SEGURANÇA: compressão {fz:.1f} N > margem '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N (teto '
                    f'{_FORCE_ABORT_LIMIT_N:.0f} N) — medição cancelada.')
                return 'force'

            progress = float(np.dot(forward_kinematics(
                self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
                - p_start, d))

            if fz > _CONTACT_ON_N:
                self._real_cmd('halt')   # descarta segmentos enfileirados
                if progress > 0.001:
                    with self._params_lock:
                        self._learned_contact_m = progress
                self.get_logger().info(
                    f'[MOVL] DESCENDING: contato em {progress*1e3:.1f} mm '
                    f'(fz={fz:.2f} N) — engatando regulação quase-estática.')
                out, fz_end = self._qs_regulate(
                    target_f, exit_tol, d, v_lim, I6,
                    budget_m=depth_m - progress,
                    stable_s=_QS_ARRIVE_S, timeout_s=_QS_TIMEOUT_S,
                    phase='DESCENDING-QS')
                if out == 'ok' or out == 'timeout':
                    if out == 'timeout':
                        self.get_logger().warn(
                            f'DESCENDING-QS: sem estabilizar em '
                            f'{_QS_TIMEOUT_S:.0f} s (fz={fz_end:.2f} N) — '
                            'entregando ao HOLD, que continua regulando.')
                    return 'ok'
                if out == 'budget':
                    self.get_logger().warn(
                        f'DESCENDING-QS: curso máximo esgotado sem sustentar '
                        f'{target_f:.2f} N (fz={fz_end:.2f} N).')
                    return 'no_contact'
                return out   # 'force' | 'stale' | 'stop'

            if progress >= depth_m - 3e-4:
                break   # curso esgotado sem contato

            # Pipeline de sub-segmentos: emite o próximo quando o robô já
            # consumiu mais da metade do enviado (fila ≤ ~2 segmentos).
            if sent_m < depth_m and sent_m - progress < self._MOVL_SEG_M * 0.5:
                seg = min(self._MOVL_SEG_M, depth_m - sent_m)
                self._real_cmd('rel', d_mm=[float(v) for v in (d * seg * 1e3)])
                sent_m += seg

            if time.time() > t_deadline:
                self._real_cmd('halt')
                self.get_logger().error(
                    f'[MOVL] DESCENDING: timeout 120 s '
                    f'(progress={progress*1e3:.1f}/{depth_m*1e3:.1f} mm) — '
                    'o robô real está executando? Abortando.')
                return 'error'
            time.sleep(_CTRL_DT)

        self.get_logger().warn(
            f'[MOVL] DESCENDING: curso máximo de {progress*1e3:.1f} mm '
            f'esgotado sem atingir {target_f:.2f} N '
            f'(fz={self._fz_corrected():.2f} N).')
        return 'no_contact'

    def _movl_slide(self, dir_world: np.ndarray, slide_lim_m: float,
                    approach_dir_eff: np.ndarray,
                    p_start: np.ndarray) -> str:
        """SLIDING em modo MovL: UM RelMovL lateral — a linearidade (altura
        constante) é garantida pela geometria do MovL, sem locks Jacobianos.
        Monitor a 33 Hz: segurança de força, afundamento, direção errada
        (frame mal calibrado) e progresso via FK.
        """
        d = np.asarray(dir_world, float)
        self._real_cmd('rel', d_mm=[float(v) for v in (d * slide_lim_m * 1e3)])
        t_deadline = time.time() + 120.0
        quiet_ticks = 0
        last_progress = 0.0
        outcome = 'ok'
        while True:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self._real_cmd('halt')
                outcome = 'stop'
                break
            was_paused = self._pause_requested.is_set()
            if not self._pause_gate():
                outcome = 'stop'
                break

            T_cur = forward_kinematics(
                self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
            progress = float(np.dot(T_cur[:3, 3] - p_start, d))

            if was_paused:
                # Pausa deu halt no segmento — re-emite o restante.
                remaining = max(0.0, slide_lim_m - progress)
                if remaining > 3e-4:
                    self._real_cmd(
                        'rel', d_mm=[float(v) for v in (d * remaining * 1e3)])

            if progress >= slide_lim_m - 3e-4:
                self.get_logger().info(
                    f'[MOVL] SLIDING: {slide_lim_m*1e3:.0f} mm (FK) atingidos.')
                break

            # Direção errada = frame mundo→DOBOT mal calibrado. Abortar
            # imediatamente — em contato, 2 mm na direção errada já é dano.
            if progress < -0.002:
                self._real_cmd('halt')
                self.get_logger().error(
                    f'[MOVL] SLIDING: TCP na direção ERRADA '
                    f'({progress*1e3:.1f} mm) — calibração de frame suspeita. '
                    'Abortando.')
                outcome = 'error'
                break

            sink_m = float(np.dot(T_cur[:3, 3] - p_start, approach_dir_eff))
            if sink_m > _SLIDE_MAX_SINK_M:
                self._real_cmd('halt')
                self.get_logger().error(
                    f'[MOVL] SLIDING: TCP afundou {sink_m*1e3:.1f} mm '
                    f'(> {_SLIDE_MAX_SINK_M*1e3:.0f} mm) — MovL deveria ser '
                    'plano; calibração de frame suspeita. Abortando.')
                outcome = 'error'
                break

            if self._force_stale_abort('SLIDING'):
                self._real_cmd('halt')
                outcome = 'stale'
                break
            fz_corr = self._fz_corrected()
            if fz_corr > _FORCE_SAFE_LIMIT_N:
                self._relieve_contact(approach_dir_eff)
                self.get_logger().error(
                    f'SEGURANÇA: compressão {fz_corr:.1f} N > margem '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N (teto '
                    f'{_FORCE_ABORT_LIMIT_N:.0f} N) — medição cancelada.')
                outcome = 'force'
                break

            # Robô parou antes do alvo (fila vazia, sem progresso) — aceita
            # parcial com aviso após ~1 s quieto.
            if abs(progress - last_progress) < 5e-5:
                quiet_ticks += 1
                if quiet_ticks > 33 and progress > 0.001:
                    self.get_logger().warn(
                        f'[MOVL] SLIDING: robô parou em {progress*1e3:.1f} mm '
                        f'de {slide_lim_m*1e3:.0f} mm — aceitando parcial.')
                    break
            else:
                quiet_ticks = 0
            last_progress = progress

            if time.time() > t_deadline:
                self._real_cmd('halt')
                self.get_logger().error('[MOVL] SLIDING: timeout 120 s.')
                outcome = 'error'
                break
            time.sleep(_CTRL_DT)

        self._settle()
        return outcome

    def _phase_hold(self, stable_s: float = _HOLD_STABLE_S,
                    timeout_s: float = _HOLD_TIMEOUT_S,
                    dwell_s: float = _HOLD_DWELL_S) -> str:
        """HOLD — regulação QUASE-ESTÁTICA mantém a compressão no setpoint,
        ESPERA a janela estável e MANTÉM por `dwell_s` (medição) antes de
        liberar.

        Critério de estabilização: mediana settled |fz − alvo| ≤ tol por
        `stable_s` s CONTÍNUOS (tol = máx(_HOLD_TOL_N, _HOLD_TOL_PCT × alvo));
        sair da banda reinicia a janela. `timeout_s` é o teto de espera.
        Dentro da banda o braço fica CONGELADO — a posição parada é o hold;
        fora dela, micro-passos move-then-measure (ver _qs_regulate).

        Retorna: 'ok' | 'force' (> 15 N) | 'stale' (célula sem dados)
                 | 'stop' (usuário).
        """
        self._set_phase('HOLD')
        # Handoff DESCENDING→HOLD: espera a força ASSENTAR (dF/dt≈0) com posição
        # travada antes de devolver o controle. Contra contato rígido (~40–120
        # N/mm medido) a inércia/lookahead herdados viravam pico; o quiet-settle
        # absorve esse transiente — é o que corta o overshoot da transição.
        self._settle_until_quiet()
        with self._params_lock:
            target_f = float(self._target_force_n)
            # Overrides do PalpationStart (avançados da GUI); None = default.
            tol_override = self._hold_tol_n
            if self._hold_stable_s is not None:
                stable_s = self._hold_stable_s
            if self._hold_timeout_s is not None:
                timeout_s = self._hold_timeout_s

        tol_n = (tol_override if tol_override is not None
                 else max(_HOLD_TOL_N, _HOLD_TOL_PCT * target_f))
        approach_dir = (self._approach_dir if self._approach_dir is not None
                        else np.array([0., 0., -1.]))
        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S

        self.get_logger().info(
            f'HOLD-QS: alvo {target_f:.2f} ± {tol_n:.2f} N  '
            f'K_est={self._k_est.value/1000:.0f} N/mm  '
            f'estável por {stable_s:.1f} s + dwell {dwell_s:.1f} s '
            f'(timeout {timeout_s:.0f} s)')

        t_start = time.time()
        out, fz = self._qs_regulate(target_f, tol_n, approach_dir, v_lim, I6,
                                    budget_m=None, stable_s=stable_s,
                                    timeout_s=timeout_s, phase='HOLD-QS')
        if out in ('force', 'stale', 'stop'):
            return out
        timed_out = out == 'timeout'

        # ── Dwell de medição: mantém o setpoint por dwell_s ──────────────
        # Mesma regulação quase-estática, agora exigindo dwell_s CONTÍNUOS
        # em banda — a janela de medição sai garantidamente estável.
        if not timed_out and dwell_s > 0.0:
            self.get_logger().info(
                f'HOLD-QS: estável — mantendo {dwell_s:.1f} s (medição).')
            out, fz = self._qs_regulate(
                target_f, tol_n, approach_dir, v_lim, I6,
                budget_m=None, stable_s=dwell_s,
                timeout_s=dwell_s + timeout_s, phase='HOLD-QS-DWELL')
            if out in ('force', 'stale', 'stop'):
                return out
            timed_out = out == 'timeout'

        if timed_out:
            self.get_logger().warn(
                f'HOLD-QS: timeout ({timeout_s:.0f} s) sem estabilizar — '
                f'fz={self._fz_corrected():.2f} N '
                f'(alvo {target_f:.2f} ± {tol_n:.2f} N). Prosseguindo: '
                'o deadbeat do SLIDING continua corrigindo.')
        else:
            self.get_logger().info(
                f'HOLD-QS: medição concluída — fz={self._fz_corrected():.2f} N '
                f'(alvo {target_f:.2f} ± {tol_n:.2f} N, estável {stable_s:.1f} s '
                f'+ dwell {dwell_s:.1f} s) em {time.time() - t_start:.1f} s.')
        return 'ok'

    def _phase_sliding(self) -> str:
        """SLIDING — movimento lateral com ALTURA (Z) TRAVADA em posição.

        O contato inicial na força alvo é estabelecido pelo DESCENDING/HOLD;
        o deslize mantém a profundidade capturada no início da fase durante
        TODO o percurso. NÃO há correção de força no trajeto: a variação da
        força ao cruzar picos e vales da textura é exatamente o sinal que o
        experimento mede — corrigi-la achataria a textura. Perda de contato
        num vale é dado, não falha (não interrompe o deslize).

        Streaming rolling-window (_SLIDE_WIN pts) combinando:
          • passo lateral constante em dir_world (velocidade do usuário)
          • lock posicional de profundidade ao longo de approach_dir
          • lock de orientação e posição perpendicular

        A única reação a força é a SEGURANÇA: > _FORCE_SAFE_LIMIT_N aborta
        (pico da textura mais alto que o plano de contato) e célula sem
        dados frescos aborta ('stale' — sem monitor de segurança confiável).

        Retorna: 'ok' | 'force' (> 15 N) | 'stale' | 'stop' (usuário)
                 | 'error'.
        """
        self._set_phase('SLIDING')
        self._settle()

        with self._params_lock:
            speed_ms   = max(0.001, self._slide_speed_mms * 1e-3)
            dir_xy     = self._slide_dir_vec.copy()
            slide_lim_m = min(float(self._target_slide_mm) / 1000.0,
                              _SLIDING_SAFETY_M)
            target_f   = float(self._target_force_n)

        approach_dir = (self._approach_dir if self._approach_dir is not None
                        else np.array([0., 0., -1.]))

        dir_world = np.array([float(dir_xy[0]), float(dir_xy[1]), 0.0])
        dn = float(np.linalg.norm(dir_world))
        if dn < 1e-9:
            self.get_logger().error('SLIDING: direção inválida.')
            return 'error'
        dir_world /= dn

        T_start = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        R0     = T_start[:3, :3].copy()
        p_start = T_start[:3, 3].copy()

        perp = np.array([-dir_world[1], dir_world[0], 0.0])
        pnorm = float(np.linalg.norm(perp))
        perp_dir = perp / pnorm if pnorm > 1e-9 else None
        p0_perp  = float(p_start @ perp_dir) if perp_dir is not None else None

        # Componente de approach_dir perpendicular a dir_world.
        # Quando approach_dir tem componente paralela ao deslizamento
        # (ex.: ferramenta levemente inclinada na direção Y), a correção
        # de profundidade disputa com o movimento lateral, variando a
        # velocidade do deslizamento. Removendo essa componente, o lock de
        # altura age apenas no subespaço ortogonal ao movimento lateral.
        _lat_comp = float(np.dot(approach_dir, dir_world))
        _adp = approach_dir - _lat_comp * dir_world
        _adp_norm = float(np.linalg.norm(_adp))
        approach_dir_eff = (_adp / _adp_norm) if _adp_norm > 1e-6 else approach_dir

        if self._movl_run:
            self.get_logger().info(
                f'[MOVL] SLIDING: RelMovL lateral de {slide_lim_m*1e3:.0f} mm '
                f'dir=({dir_world[0]:+.0f},{dir_world[1]:+.0f},0) — altura '
                'constante por geometria (velocidade = SpeedFactor global).')
            return self._movl_slide(dir_world, slide_lim_m,
                                    approach_dir_eff, p_start)

        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        dt = _CTRL_DT

        dist_planned_m   = 0.0   # distância planejada acumulada (não depende de FK)
        step_m = speed_ms * dt   # deslocamento por tick no plano

        self.get_logger().info(
            f'SLIDING: speed={speed_ms*1e3:.1f} mm/s  '
            f'dir=({dir_world[0]:+.0f},{dir_world[1]:+.0f},0)  '
            f'alvo={slide_lim_m*1e3:.0f} mm  '
            f'Z travado no plano de contato ({target_f:.2f} N no início)  '
            f'approach_eff=({approach_dir_eff[0]:+.3f},'
            f'{approach_dir_eff[1]:+.3f},{approach_dir_eff[2]:+.3f})')

        outcome = 'ok'
        while True:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                outcome = 'stop'
                break
            if not self._pause_gate():
                outcome = 'stop'
                break

            t0 = time.time()

            # Verificação 1: distância planejada acumulada (determinístico).
            # Não depende de atraso do /joint_states — para assim que os
            # waypoints suficientes foram enviados ao JTC.
            if dist_planned_m >= slide_lim_m:
                self.get_logger().info(
                    f'SLIDING: {slide_lim_m*1e3:.0f} mm planejados — parando.')
                break

            # Verificação 2: posição real via FK (segurança extra).
            q = self._q_now()
            T_cur = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)
            progress = float(np.dot(T_cur[:3, 3] - p_start, dir_world))
            if progress >= slide_lim_m:
                self.get_logger().info(
                    f'SLIDING: {slide_lim_m*1e3:.0f} mm (FK) atingidos.')
                break

            # Verificação 3: afundamento máximo. Com o lock de altura o TCP
            # não deveria descer do plano inicial; exceder _SLIDE_MAX_SINK_M
            # indica falha (clipping de juntas, FK/estado inconsistente) —
            # terminar em vez de continuar arando a amostra.
            sink_m = float(np.dot(T_cur[:3, 3] - p_start, approach_dir_eff))
            if sink_m > _SLIDE_MAX_SINK_M:
                self.get_logger().warn(
                    f'SLIDING: TCP afundou {sink_m*1e3:.1f} mm '
                    f'(> {_SLIDE_MAX_SINK_M*1e3:.0f} mm) abaixo do plano '
                    f'inicial após {progress*1e3:.1f} mm — terminando o '
                    'deslize.')
                break

            # Força só como SEGURANÇA: sem célula fresca não há monitor
            # confiável (aborta); acima da margem, pico da textura mais alto
            # que o plano de contato — trava e cancela.
            if self._force_stale_abort('SLIDING'):
                outcome = 'stale'
                break
            fz_corr = self._fz_corrected()

            if fz_corr > _FORCE_SAFE_LIMIT_N:
                self._relieve_contact(approach_dir_eff)   # recua NA HORA
                self.get_logger().error(
                    f'SEGURANÇA: compressão {fz_corr:.1f} N > margem '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N (teto {_FORCE_ABORT_LIMIT_N:.0f} N) '
                    f'— medição cancelada.')
                outcome = 'force'
                break

            # ── Rolling-window de _SLIDE_WIN waypoints ──────────────────
            msg = JointTrajectory()
            msg.joint_names = list(_ARM_JOINTS)
            q_iter = q.copy()
            T_iter = T_cur
            singular = False

            for k in range(1, _SLIDE_WIN + 1):
                tw = np.zeros(6)
                # Passo lateral — limita o último passo para não ultrapassar alvo
                remaining = max(0.0, slide_lim_m - dist_planned_m - (k - 1) * step_m)
                lateral   = min(step_m, remaining)
                tw[:3] = dir_world * lateral
                # Lock de orientação
                R_err = R0 @ T_iter[:3, :3].T
                tw[3:] = _ORI_GAIN * 0.5 * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ])
                # Lock de altura ⊥ ao deslizamento: mantém a profundidade do
                # plano de contato (p_start, onde o HOLD deixou a força alvo)
                # durante TODO o percurso. A força fica LIVRE para variar com
                # a textura — é o sinal medido.
                depth_err = float(np.dot(
                    p_start - T_iter[:3, 3], approach_dir_eff))
                tw[:3] += _Z_CORR_GAIN * depth_err * approach_dir_eff
                # Lock perpendicular — sem ganho na direção transversal ao sliding
                if perp_dir is not None and p0_perp is not None:
                    perp_err = p0_perp - float(T_iter[:3, 3] @ perp_dir)
                    tw[:3] += _Z_CORR_GAIN * perp_err * perp_dir

                J_k = jacobian(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
                try:
                    dq_k = J_k.T @ np.linalg.solve(
                        J_k @ J_k.T + _JAC_LAM**2 * I6, tw)
                except np.linalg.LinAlgError:
                    singular = True
                    break

                q_next = np.clip(q_iter + dq_k, JOINT_MIN, JOINT_MAX)
                vel_k  = np.clip((q_next - q_iter) / dt, -v_lim, v_lim)
                if k == _SLIDE_WIN:
                    vel_k = np.zeros(6)

                pt = JointTrajectoryPoint()
                pt.positions  = [float(x) for x in q_next]
                pt.velocities = [float(x) for x in vel_k]
                t_k = k * dt
                pt.time_from_start = Duration(
                    sec=int(t_k), nanosec=int((t_k - int(t_k)) * 1e9))
                msg.points.append(pt)
                q_iter = q_next
                if k < _SLIDE_WIN:
                    T_iter = forward_kinematics(q_iter, T_end=T_TOUCH_TOOL_ATTACH)

            if singular:
                self.get_logger().warn('SLIDING: Jacobiano singular — passo descartado.')
            elif msg.points:
                self._arm_traj_pub.publish(msg)
                # A janela desliza 1 passo por tick (cada mensagem SUBSTITUI a
                # anterior no JTC — só ~1 segmento executa antes da próxima).
                # Contar os _SLIDE_WIN pontos do lookahead triplicava o
                # planejado e o deslize terminava com ~1/3 da distância pedida.
                dist_planned_m += min(step_m,
                                      max(0.0, slide_lim_m - dist_planned_m))

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        self._settle()
        return outcome

    # RETRACT removido: o experimento (toque e deslizamento) vai DIRETO à
    # home ao terminar e entre ciclos — ver _retreat_and_home e _run_protocol.
    # O _phase_goto_home já afasta da superfície ao subir.

    # ──────────────────────────────────────────────────────────────────
    # Orquestração
    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._stop_requested.set()
        if self._protocol_thread is not None:
            self._protocol_thread.join(timeout=2.0)
        super().destroy_node()

    def _retreat_and_home(self, final_phase: str) -> None:
        """Término com SUCESSO: retorna DIRETO à home lentamente, sem RETRACT.

        A pedido do usuário, ao terminar o teste (toque ou deslizamento) o
        braço vai direto para a home (≤ home_speed_rad_s por junta) — o
        _phase_goto_home já afasta da superfície ao subir para a home.
        """
        self._phase_goto_home()
        self._set_phase(final_phase)

    def _abort_to_home(self) -> None:
        """Falha do experimento (qualquer motivo): sem RETRACT — retorna
        direto à home lentamente (≤ home_speed_rad_s por junta) e marca
        ABORTED."""
        self._phase_goto_home()
        self._set_phase('ABORTED')

    def _run_protocol(self):
        self._busy.set()
        # Snapshot do modo MovL para TODO o experimento — não muda no meio
        # de uma fase se a GUI desconectar (as fases abortam por timeout).
        self._movl_run = bool(self._movl_param and self._movl_avail)
        if self._movl_param and not self._movl_avail:
            self.get_logger().info(
                '[MOVL] indisponível (robô real não conectado em MIRROR) — '
                'usando streaming clássico ao Gazebo.')
        if self._movl_run:
            self.get_logger().info(
                '[MOVL] experimento em modo MovL: robô real executa '
                'MovJ/RelMovL; o sim espelha o feedback.')
            self._real_cmd('run_begin')
        try:
            with self._params_lock:
                repeats = int(self._repeats)
                mode = self._mode
            self._cycles_total = repeats
            # Em TOUCH, cada "ciclo" é um toque (descida → hold → recuo).
            label = 'TOQUE' if mode == 'TOUCH' else 'CICLO'

            for cycle in range(1, repeats + 1):
                self._cycle = cycle
                if repeats > 1:
                    self.get_logger().info(
                        f'[{label}] {cycle}/{repeats}')

                if not self._phase_goto_home():
                    self._set_phase('ABORTED'); return

                out = self._phase_descending()
                if out in ('force', 'no_contact', 'stale'):
                    self._abort_to_home(); return
                if out != 'ok':   # stop do usuário → para no lugar
                    self._set_phase('ABORTED'); return

                out = self._phase_hold()
                if out in ('force', 'stale'):
                    self._abort_to_home(); return
                if out != 'ok':
                    self._set_phase('ABORTED'); return

                # Modo TOUCH: só toca a mesa com força controlada (DESCENDING
                # + HOLD) e recua — sem deslizamento lateral.
                if mode != 'TOUCH':
                    out = self._phase_sliding()
                    if out in ('force', 'error', 'stale'):
                        self._abort_to_home(); return
                    if out != 'ok':
                        self._set_phase('ABORTED'); return

                if cycle < repeats:
                    # Entre ciclos (coleta de dados): vai DIRETO à home, sem
                    # RETRACT — o HOME já afasta da superfície ao subir, e o
                    # próximo ciclo refaz a re-aproximação a partir da home.
                    if not self._phase_goto_home():
                        self._set_phase('ABORTED'); return
                    # Stop pedido durante o retorno → não inicia o próximo.
                    if self._stop_requested.is_set():
                        self._stop_requested.clear()
                        self._set_phase('ABORTED'); return

            self._retreat_and_home('DONE')
            time.sleep(0.5)
            self._set_phase('IDLE')
        finally:
            if self._movl_run:
                self._real_cmd('run_end')
                self._movl_run = False
            self._cycle = 0
            self._cycles_total = 1
            self._busy.clear()


def main(args=None):
    rclpy.init(args=args)
    node = TactileExplorer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
