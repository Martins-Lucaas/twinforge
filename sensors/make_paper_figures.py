#!/usr/bin/env python3
"""
Gera as figuras do artigo da célula de carga a partir dos dados reais em
sensors/Data/ e de load_cell_calib.json.

Saídas (em ../images/):
    calibration_curve.png   - curva de calibração (força x tensão) + resíduos
    force_time_cycles.png   - força x tempo dos 3 ciclos, colorido por fase
    hysteresis_loop.png     - força x deslocamento (tcp_z): carga vs descarga
    creep_recovery.png      - deriva da força durante o HOLD (deslocamento fixo)

Uso:
    python make_paper_figures.py [caminho_do_samples.csv]

Se nenhum CSV for passado, usa a coleta padrão abaixo (DEFAULT_RUN).
Requer: numpy, matplotlib.
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
HERE = Path(__file__).resolve().parent
DATA_DIR = HERE / "Data"
IMG_DIR = HERE.parent / "images"
CALIB_JSON = HERE / "load_cell_calib.json"
DEFAULT_RUN = "20260713_155008"          # coleta de 3 ciclos, setpoint 6.5 N

# Codificação da coluna 'phase' (ver summary.json / firmware)
PHASE_NAMES = {-1: "IDLE", 0: "HOME", 1: "DESCENDING", 2: "HOLD"}
PHASE_COLORS = {-1: "#bbbbbb", 0: "#8c9bb0", 1: "#d1720a", 2: "#1a9641"}
G0 = 9.80665


def _load_samples(csv_path):
    """Lê o samples.csv e devolve arrays numpy das colunas usadas."""
    t, force, phase, cycle, tcpz, setp = [], [], [], [], [], []
    with open(csv_path, newline="") as fh:
        for row in csv.DictReader(fh):
            try:
                t.append(float(row["t_rel_s"]))
                force.append(float(row["force_net_n"]))
                phase.append(int(float(row["phase"])))
                cycle.append(int(float(row["cycle"])))
                tcpz.append(float(row["tcp_z"]))
                setp.append(float(row["setpoint_n"]))
            except (KeyError, ValueError):
                continue
    return (np.array(t), np.array(force), np.array(phase),
            np.array(cycle), np.array(tcpz), np.array(setp))


# ---------------------------------------------------------------------------
def fig_calibration():
    """Curva de calibração + resíduos a partir dos pontos reais do JSON."""
    calib = json.loads(CALIB_JSON.read_text())
    pts = sorted(calib["points"], key=lambda p: p["v_sensor"])
    v = np.array([p["v_sensor"] for p in pts])
    f = np.array([p["force_n"] for p in pts])

    # Ajuste linear por mínimos quadrados: F = S*v + F0
    S, F0 = np.polyfit(v, f, 1)
    f_hat = S * v + F0
    resid = f - f_hat
    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((f - f.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot
    rmse = float(np.sqrt(ss_res / len(v)))

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(6.4, 5.0), sharex=True,
        gridspec_kw={"height_ratios": [3, 1]})

    vv = np.linspace(0, v.max() * 1.08, 100)
    ax1.plot(vv, S * vv + F0, "-", color="#1f6fb2",
             label=fr"$F = {S:.3f}\,v {F0:+.3f}$")
    ax1.plot(v, f, "o", color="#d1720a", ms=7, label="Calibration points")
    ax1.set_ylabel("Applied force (N)")
    ax1.set_title(f"Load-cell calibration   "
                  fr"($S={S:.2f}$ N/V,  $R^2={r2:.4f}$,  RMSE$={rmse:.3f}$ N)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper left")

    ax2.axhline(0, color="#888", lw=0.8)
    ax2.stem(v, resid, linefmt="#d1720a", markerfmt="o", basefmt=" ")
    ax2.set_xlabel("Amplifier-referred voltage $v$ (V)")
    ax2.set_ylabel("Residual (N)")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    out = IMG_DIR / "calibration_curve.png"
    fig.savefig(out, dpi=200)
    plt.close(fig)
    print(f"[ok] {out.name}  (S={S:.3f} N/V, R2={r2:.5f}, RMSE={rmse:.4f} N, "
          f"max|resid|={np.max(np.abs(resid)):.4f} N)")


def fig_force_time(t, force, phase, setp):
    """Força x tempo, colorida por fase (reproduz o plot da coleta)."""
    fig, ax = plt.subplots(figsize=(7.2, 3.2))
    for ph, name in PHASE_NAMES.items():
        m = phase == ph
        if not m.any():
            continue
        ax.plot(t[m], force[m], ".", ms=1.5, color=PHASE_COLORS[ph], label=name)
    sp = float(np.median(setp[setp > 0])) if np.any(setp > 0) else np.nan
    if np.isfinite(sp):
        ax.axhline(sp, ls="--", color="#b22222", lw=1,
                   label=f"setpoint {sp:.1f} N")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Compression force (N)")
    ax.set_title("Palpation cycles - force vs. time")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", markerscale=6, ncol=2, fontsize=8)
    fig.tight_layout()
    out = IMG_DIR / "force_time_cycles.png"
    fig.savefig(out, dpi=200)
    plt.close(fig)
    print(f"[ok] {out.name}")


def fig_force_penetration(force, phase, cycle, tcpz, f_contact=0.1):
    """Curva força x penetração na região de CONTATO (zoom), por ciclo.

    Os dados são 'aproximar-e-segurar': quase todo o curso é aproximacao em
    espaco livre (forca ~0) e nao ha varredura de descarga, entao NAO existe
    laco de histerese mensuravel. Aqui mostramos a curva de carga forca x
    penetracao no contato (rigidez de contato) e a repetibilidade entre ciclos.
    A penetracao e re-zerada no primeiro contato (forca > f_contact) de cada ciclo.
    """
    fig, ax = plt.subplots(figsize=(6.0, 4.2))
    z_mm = tcpz * 1000.0
    cyc_ids = sorted(c for c in set(cycle) if c > 0)
    colors = plt.cm.viridis(np.linspace(0, 0.8, len(cyc_ids)))
    for c, col in zip(cyc_ids, colors):
        m = (cycle == c) & np.isin(phase, (1, 2)) & (force > f_contact)
        if m.sum() < 10:
            continue
        z_c = z_mm[m]
        pen = z_c.max() - z_c            # penetracao (mm) a partir do 1o contato
        ax.plot(pen, force[m], ".", ms=2, color=col, label=f"cycle {c}")
    ax.set_xlabel("Penetration from first contact (mm)")
    ax.set_ylabel("Compression force (N)")
    ax.set_title("Contact force-penetration curve (loading)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8, markerscale=4)
    fig.tight_layout()
    out = IMG_DIR / "force_penetration.png"
    fig.savefig(out, dpi=200)
    plt.close(fig)
    print(f"[ok] {out.name}")


def fig_creep(t, force, phase, cycle, setp):
    """Deriva da força durante o HOLD (deslocamento fixo), por ciclo."""
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    cyc_ids = sorted(c for c in set(cycle) if c > 0)
    colors = plt.cm.plasma(np.linspace(0, 0.75, len(cyc_ids)))
    for c, col in zip(cyc_ids, colors):
        m = (cycle == c) & (phase == 2)
        if m.sum() < 10:
            continue
        th = t[m] - t[m].min()
        ax.plot(th, force[m], "-", lw=0.9, color=col, label=f"cycle {c}")
    sp = float(np.median(setp[setp > 0])) if np.any(setp > 0) else np.nan
    if np.isfinite(sp):
        ax.axhline(sp, ls="--", color="#b22222", lw=1,
                   label=f"setpoint {sp:.1f} N")
    ax.set_xlabel("Time in hold (s)")
    ax.set_ylabel("Compression force (N)")
    ax.set_title("Force during constant-displacement hold (creep)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    out = IMG_DIR / "creep_recovery.png"
    fig.savefig(out, dpi=200)
    plt.close(fig)
    print(f"[ok] {out.name}")


def main():
    IMG_DIR.mkdir(exist_ok=True)
    if len(sys.argv) > 1:
        csv_path = Path(sys.argv[1])
    else:
        csv_path = DATA_DIR / f"{DEFAULT_RUN}__samples.csv"
    if not csv_path.exists():
        sys.exit(f"CSV nao encontrado: {csv_path}")

    print(f"Dados: {csv_path.name}")
    t, force, phase, cycle, tcpz, setp = _load_samples(csv_path)

    fig_calibration()
    fig_force_time(t, force, phase, setp)
    fig_force_penetration(force, phase, cycle, tcpz)
    fig_creep(t, force, phase, cycle, setp)
    print(f"\nFiguras salvas em: {IMG_DIR}")


if __name__ == "__main__":
    main()
