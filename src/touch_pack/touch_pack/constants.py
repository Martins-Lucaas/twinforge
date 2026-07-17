"""
constants.py — Constantes compartilhadas do touch_pack.

Regra: valores usados por MAIS de um módulo moram aqui; valores privados
de um único módulo ficam nele.
"""
from __future__ import annotations

import math
import os

# ── Cadeia do braço CR10 (convenção URDF) ────────────────────────────────────
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Pose "apontar para a mesa": home default da GUI e seed POINTING do explorer.
POINTING_SEED_DEG = {'joint1': 0.0, 'joint2': 0.0, 'joint3': -90.0,
                     'joint4': 0.0, 'joint5': 90.0, 'joint6': 0.0}

# ── Mão COVVI — juntas primárias ─────────────────────────────────────────────
HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# Pose POINTING (palpação com o Index estendido).
HAND_POINT_DEG = {'Thumb': 30.0, 'Index': 0.0, 'Middle': 80.0,
                  'Ring': 80.0, 'Little': 80.0, 'Rotate': 0.0}
HAND_POINTING_RAD = {j: math.radians(v) for j, v in HAND_POINT_DEG.items()}

# ── Controle de força ────────────────────────────────────────────────────────
# Limite de segurança: medição CANCELADA se a compressão exceder este valor.
# Com a célula de 100 kg (≥ LOAD_CELL_RATED_N) o limite NÃO protege mais a
# célula — protege a amostra, a mesa e o robô; os valores continuam os dos
# ensaios do artigo (setpoints 1–10 N).
FORCE_ABORT_LIMIT_N = 15.0
# Setpoint máximo selecionável na GUI.
FORCE_SETPOINT_MAX_N = 10.0

# ── Identidade da célula de carga vigente ────────────────────────────────────
# 17/07/2026: célula CSA/ZL tipo S de 100 kg (50,8×19,1×76,2 mm, M12×1,75),
# montada AXIALMENTE entre os acopladores impressos — a barra 5 kg cantilever
# saiu e o TCP MUDOU junto: agora é coaxial ao flange, (0, 0, +206,7) mm
# (kinematics.T_TOUCH_TOOL_ATTACH). O firmware não muda — só transmite a
# tensão da ponte. A SENSIBILIDADE muda (~20× menos V por N): slope/intercept
# pertencem à CÉLULA, então toda troca exige recalibrar na aba Calibration da
# GUI (a assinatura voltage_scale/offset já força isso).
LOAD_CELL_RATED_KG = 100.0
LOAD_CELL_RATED_N  = LOAD_CELL_RATED_KG * 9.80665   # ≈ 980,7 N

# ── Célula de carga (XIAO ESP32S3 + HX711, telemetria UDP) ───────────────────
LOAD_CELL_UDP_PORT = 8080
# Amostra (little-endian, 12 bytes), espelhada no struct Sample do firmware
# (sensors/ForceDriver/src/main.cpp):
#   uint32 seq      — contador por amostra (salto = amostras perdidas)
#   uint32 t_us     — micros() da ESP (relógio de sincronização)
#   float  v_sensor — tensão da ponte ×PGA (V); filtro pesado fica no PC
LOAD_CELL_SAMPLE_FMT = '<IIf'
# Amostras por datagrama (1 no firmware atual; o receiver aceita qualquer
# múltiplo de 12 B).
LOAD_CELL_BATCH_N = 1
# Taxa nominal do HX711 (pino RATE: GND = 10 Hz, VDD = 80 Hz) — só chute
# inicial do filtro; o dt real medido pelo t_us é quem manda.
LOAD_CELL_NOMINAL_RATE_HZ = 80.0

# Auto-descoberta: o force_receiver manda um "hello" periódico ao IP fixo do
# ESP e passa a receber a telemetria por unicast (broadcast em WiFi perde
# muito). Espelhado no firmware.
LOAD_CELL_ESP_IP        = '192.168.5.105'   # IP estático do ESP — rede do LAB; em casa ("Martins 6") era .6.105
LOAD_CELL_DISCOVERY_PORT = 8090
LOAD_CELL_DISCOVERY_MAGIC = b'FRCV'

# Transporte SERIAL paralelo (sempre ativo): o firmware imprime cada amostra
# na USB CDC como "F,<seq>,<t_us>,<v_sensor>" e a palpation_gui lê a porta
# direto (lc_serial.py). A GUI deduplica: ignora a serial enquanto o UDP está
# fresco. Espelhado no firmware.
LOAD_CELL_SERIAL_BAUD   = 115200
LOAD_CELL_SERIAL_PREFIX = 'F,'
# VID USB da Espressif (XIAO): identifica a porta da célula e a distingue do
# STM32 do touch sensor — o auto-detect do toque EXCLUI este VID.
LOAD_CELL_USB_VID = 0x303A

# Conversão counts→v_sensor aplicada no firmware (v = counts·SCALE − OFFSET).
# ESPELHO EXATO de COUNTS_TO_V/V_OFFSET do main.cpp — se um mudar, o outro
# TEM de mudar junto. Os dois valores formam a ASSINATURA da configuração de
# hardware: a GUI grava ambos no load_cell_calib.json ao calibrar, e GUI +
# force_receiver RECUSAM calibração com assinatura divergente (slope/intercept
# de outra escala dariam força errada silenciosamente).
LC_FW_VOLTAGE_SCALE  = 3.3 / (1 << 24)   # AVDD/2²⁴ ≈ 1.9670e-7 V/count
LC_FW_VOLTAGE_OFFSET = 0.0

# ── Touch sensor (STM32 → PC plotter → UDP) ──────────────────────────────────
# Porta DIFERENTE da célula: o receptor não filtra remetente, na mesma porta
# os fluxos se misturariam.
TOUCH_SENSOR_UDP_PORT = 8081
# Payload (little-endian, 8 bytes): uint32 seq + float valor. Espelhado no
# plotter standalone (sensors/Touch_sensor).
TOUCH_PAYLOAD_FMT = '<If'
# Broadcast do I_final reemitido pelo TouchSensorSource a cada TOTAL.
TOUCH_UDP_BROADCAST_IP = '192.168.5.255'
# Relay do frame COMPLETO do toque (linhas brutas do STM32) para PCs sem USB:
# um PC remoto faz bind aqui e injeta as linhas no mesmo parser. Porta
# separada porque a 8081 carrega só o escalar '<If'.
TOUCH_FRAME_UDP_PORT = 8082

# Idade máxima de uma amostra para entrar no par sincronizado (s).
SYNC_MAX_AGE_S = 0.25

# ── Tátil completo em ROS (frame ADC + eventos) p/ o CSV unificado ───────────
# A GUI republica o frame de taxels e cada evento de spike para o
# palpation_logger juntar tudo num único CSV.
TOUCH_ADC_TOPIC   = '/touch_sensor/adc'          # std_msgs/Int32MultiArray
TOUCH_EVENT_TOPIC = '/touch_sensor/spike_event'  # std_msgs/String: RA|SA|CN_MM|CN_RA|CN_SA
TOUCH_TAXELS_DEFAULT = 25                         # grade 5×5
TOUCH_EVENT_TYPES = ('RA', 'SA', 'CN_MM', 'CN_RA', 'CN_SA')

# ── Códigos numéricos das fases no CSV unificado ─────────────────────────────
# RETRACT dobrado no HOME.
PHASE_CODES = {
    'IDLE': -1, 'HOME': 0, 'DESCENDING': 1, 'HOLD': 2, 'SLIDING': 3,
    'RETRACT': 0, 'DONE': 4, 'ABORTED': 5,
}
PHASE_NAMES = {-1: 'IDLE', 0: 'HOME', 1: 'DESCENDING', 2: 'HOLD',
               3: 'SLIDING', 4: 'DONE', 5: 'ABORTED'}

# ── Arquivos de configuração persistente (~/.config/touch_pack/) ─────────────
CONFIG_DIR            = os.path.expanduser('~/.config/touch_pack')
HOME_POSE_FILE        = os.path.join(CONFIG_DIR, 'home_pose.json')
ROBOT_CONFIG_FILE     = os.path.join(CONFIG_DIR, 'robot.json')
LC_CALIB_FILE         = os.path.join(CONFIG_DIR, 'load_cell_calib.json')
POSES_FILE            = os.path.join(CONFIG_DIR, 'poses.json')
PALPATION_PARAMS_FILE = os.path.join(CONFIG_DIR, 'palpation_params.json')

# ── Saída dos runs ───────────────────────────────────────────────────────────
# CSVs gravados em <raiz_do_repo>/sensors/Data. Override: TOUCH_PACK_DATA_DIR.
def _resolve_repo_root() -> str | None:
    """Sobe a partir deste arquivo até achar um diretório com `sensors/` —
    funciona do código-fonte (src/...) e do espaço instalado (install/...).
    None se o pacote estiver instalado fora da árvore do repo."""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(d, 'sensors')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


_REPO_ROOT = _resolve_repo_root()


def _resolve_runs_dir() -> str:
    env = os.environ.get('TOUCH_PACK_DATA_DIR')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    if _REPO_ROOT:
        return os.path.join(_REPO_ROOT, 'sensors', 'Data')
    return os.path.expanduser('~/touch_pack_runs')


RUNS_DIR = _resolve_runs_dir()

# ── Calibração da célula compartilhada via git ───────────────────────────────
# A calibração pertence ao SENSOR, não ao PC: uma cópia versionada vive no
# repo para toda máquina que clonar já vir calibrada. Leitura: a local
# (~/.config) tem precedência; escrita vai sempre para a local (e é espelhada
# no repo para virar diff de commit).
LC_CALIB_REPO_FILE = (os.path.join(_REPO_ROOT, 'sensors', 'load_cell_calib.json')
                      if _REPO_ROOT else None)


def lc_calib_read_path() -> str:
    """Caminho de onde LER a calibração (local > repo > local inexistente)."""
    if os.path.exists(LC_CALIB_FILE):
        return LC_CALIB_FILE
    if LC_CALIB_REPO_FILE and os.path.exists(LC_CALIB_REPO_FILE):
        return LC_CALIB_REPO_FILE
    return LC_CALIB_FILE
