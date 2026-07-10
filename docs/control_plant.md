# Planta de funcionamento — Robô CR10 + célula de carga (modo Palpação)

Documento gerado a partir das análises dos dados de `sensors/Data` e do código
de `src/touch_pack`. Descreve a **malha de controle de força** (visão de
controle) e a **cadeia física de nós/tópicos** que a implementa, além dos
pontos críticos e das correções já aplicadas.

---

## 1. Malha de controle de força (closed-loop)

```
                        ┌──────────────── CONTROLADOR (tactile_explorer) ───────────────┐
                        │                                                               │
 r = força alvo         │   e        _ForcePID (PI)            v_cmd          dq         │
 (force_n, GUI)         │  erro   ┌──────────────────┐   (m/s, ±5mm/s)   (rad, juntas)   │
       │                │         │ v = Kp·e + Ki·∫e │                                   │
       ▼                ▼         │  ∫ só EM contato │      ┌──────────┐    ┌─────────┐   │
     ( Σ )─────────────►(−)──────►│  (anti-windup)   ├─────►│ Jacobiano│───►│ clip a  │───┼─► JTC
       ▲                          └──────────────────┘      │  J⁻¹·tw  │    │ v_lim   │   │  33 Hz
       │                                                    └──────────┘    └─────────┘   │
       │ fz (medida)             loop @ 33 Hz (_CTRL_DT = 30 ms)                           │
       │                         └─────────────────────────────────────────────────────────┘
       │                                                                          │
       │  ┌──────────────── SENSOR (cadeia de medição) ──────────────┐    ┌──────────────────┐
       │  │ calib (slope/ic) ◄ One-Euro(4 Hz,β7)+mediana ◄ ADC ◄ amp │    │   ROBÔ CR10       │
       └──┤ + TARE + auto-zero    ~40 ms de atraso        12-bit     │◄───┤  (ATUADOR+PLANTA) │
          │ → /load_cell/force_net                                   │  F │  juntas → TCP      │
          └───────────────────────────────────────────────────────────┘    │  → CONTATO rígido │
                                                                           │  (<1 mm ≈ vários N)│
                                                                           │  + superfície      │
                                                                           └──────────────────┘
```

| Bloco da malha | Implementação |
|---|---|
| Setpoint `r` + ganhos | `palpation_gui` → msg `PalpationStart` (`/palpation/start`); Kp/Ki/Kd **÷1000** mm/s/N→m/s/N |
| Controlador PI | `tactile_explorer._ForcePID` (HOLD/SLIDING) |
| Atuador | `J⁻¹` → `/cr10_group_controller/joint_trajectory` → `mirror_node`/`real_driver` (ServoJ 33 Hz) |
| Planta | braço CR10 + **rigidez do contato** + superfície |
| Sensor | célula 5 kg → amp → divisor 3.24 → ADC ESP32 → `force_receiver` (filtro) → `palpation_gui` (tare) → `/load_cell/force_net` |

---

## 2. Cadeia física / pipeline de nós e tópicos

```
 HARDWARE                         WiFi 192.168.5.x                 ROS 2
┌────────────┐   mV    ┌──────────────────────┐  UDP:8080   ┌──────────────────────┐
│ Célula 5kg │────────►│ ESP32  (ForceDriver) │ ~100 pkt/s  │ force_receiver_node  │
│ + amp      │ GPIO34  │ ADC 12b, oversample  │────────────►│ mediana5 + One-Euro  │
│ div. 3.24  │         │ 1 kHz, batch ×10     │  '<IIf'     │ + calib(slope/ic)    │
└────────────┘         └──────────────────────┘             └──────────┬───────────┘
                                                                       │ /load_cell/voltage(_raw), /force
                                                                       ▼
┌──────────────────────────────────────────────────────┐  /load_cell/force_net   ┌────────────────┐
│ palpation_gui                                         │◄────── tare+auto-zero ──│  (republica)   │
│  • setpoint, Kp/Ki/Kd, modo (TOUCH/SLIDE)             │                         └────────────────┘
│  • /palpation/start ─────────┐                        │
│  • mostra /palpation/status  │                        │            /load_cell/force_net
└──────────────────────────────┼────────────────────────┘                 │   │   │
                              ▼                                            │   │   ▼
                   ┌────────────────────┐  /cr10.../joint_trajectory  ┌────┘   │  ┌──────────────────┐
                   │ tactile_explorer   │────────(33 Hz)─────────────►│ mirror_node/real_driver →   │
                   │  state machine +   │                             │ CR10 (real)  ou  Gazebo     │
                   │  PI de força       │◄────────────────────────────│  → TCP → contato → célula   │
                   │  /palpation/status │      /joint_states          └─────────────────────────────┘
                   └─────────┬──────────┘
                             │ /load_cell/force_net + /joint_states + /palpation/status
                             ▼
                   ┌────────────────────┐        ┌──────────────────────────────────┐
                   │ palpation_logger   │        │ force_sync_node                  │
                   │ → CSV (samples)    │        │ force_net + touch → /touch_sync  │
                   └────────────────────┘        └──────────────────────────────────┘
```

### Tópicos principais
| Tópico | Tipo | Papel |
|---|---|---|
| `/load_cell/voltage_raw` | Float32 | tensão crua (display/diagnóstico) |
| `/load_cell/voltage` | Float32 | tensão filtrada (One-Euro) |
| `/load_cell/force` | Float32 | força calibrada **sem** tare (display) |
| `/load_cell/force_net` | Float32 | **força usada no controle** (tare + auto-zero, +compressão) |
| `/palpation/start` | PalpationStart | setpoint + ganhos + modo (RELIABLE, TRANSIENT_LOCAL) |
| `/palpation/status` | PalpationStatus | fase + força medida |
| `/cr10_group_controller/joint_trajectory` | JointTrajectory | streaming 33 Hz ao robô |
| `/touch_sync/data` | SyncedTouch | par força+toque sincronizado (100 Hz) |

---

## 3. Máquina de estados (sem RETRACT)

```
 IDLE ─► HOME ─► DESCENDING ─► HOLD ─► SLIDING ─► HOME ─► IDLE
                  (rampa de      (PI    (PI+Jac    (direto,
                   frenagem)     força)  lateral)   sem retract)
   falha/abort ───────────────────────────────────► HOME (ABORTED)
```

`DESCENDING` desce até `fz ≥ alvo` (com rampa de frenagem perto do contato).
`HOLD` regula no setpoint até estabilizar. `SLIDING` desliza com PI de força
perpendicular. Sucesso e falha vão **direto à home** (sem fase RETRACT).

---

## 4. Pontos críticos e correções aplicadas

| Ponto | Status | Correção |
|---|---|---|
| 🔴 Atraso do sensor (~150 ms @ mincutoff 1 Hz) na malha de 33 Hz | ✅ corrigido | `ONE_EURO_MINCUTOFF` 1→**4 Hz** (~40 ms) em `force_receiver_node.py` |
| 🔴 Rigidez do contato (<1 mm ≈ vários N) → overshoot ao bater | ✅ mitigado | rampa de frenagem no `DESCENDING` (`v_max→v_min`) + atraso do filtro reduzido |
| 🟠 Ganho mal escalado (slider mm/s/N ÷1000 → SI) | ✅ corrigido | hints da GUI explicitam unidade/÷1000; "não copiar valor do log" |
| 🟠 Saturação do ADC vs abort | ✅ resolvido | com a calibração confirmada (slope 0.0505), saturação ≈ 211 N ≫ 15 N |
| 🟢 Windup na perda de contato | ✅ corrigido | `_ForcePID` integra só `in_contact` |

### Garantia de força ≤ 15 N (requisito)
O teto absoluto é `FORCE_ABORT_LIMIT_N = 15 N`. Como a leitura tem atraso e o
braço tem inércia, a parada dispara **com margem** em
`_FORCE_SAFE_LIMIT_N = 12 N` (3 N abaixo do teto). Em todas as fases
(`DESCENDING`/`HOLD`/`SLIDING`):
1. a força é checada **antes** de comandar qualquer movimento;
2. ao cruzar 12 N, o nó **trava a posição na hora** (`_settle`, velocidade zero)
   e aborta direto para a home.

Assim o overshoot residual fica abaixo dos 15 N — o sistema nunca aplica mais
que o teto.

### Parâmetros-chave
| Parâmetro | Valor | Local |
|---|---|---|
| Loop de controle | 33 Hz (`_CTRL_DT` = 30 ms) | `tactile_explorer.py` |
| Saída máx. do PID | ±5 mm/s (`_PID_V_MAX_MS`) | `tactile_explorer.py` |
| Anti-windup do integrador | ±5 N·s (`_PID_I_MAX_Ns`) | `tactile_explorer.py` |
| Margem de segurança | 12 N (`_FORCE_SAFE_LIMIT_N`) | `tactile_explorer.py` |
| Teto absoluto | 15 N (`FORCE_ABORT_LIMIT_N`) | `constants.py` |
| Setpoint máx. | 10 N (`FORCE_SETPOINT_MAX_N`) | `constants.py` |
| Filtro da célula | One-Euro mincutoff 4 Hz, β 7 | `force_receiver_node.py` |
| Calibração | slope 0.0505, R² 0.876 | `load_cell_calib.json` |
| Conversão de ganho | mm/s/N ÷1000 → m/s/N | `palpation_gui.py` |
