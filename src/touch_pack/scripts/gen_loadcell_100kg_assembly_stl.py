#!/usr/bin/env python3
"""Gera meshes/CelulaDeCarga_100kg_Montagem.stl (célula tipo S CSA/ZL 100 kg).

A célula física é uma CSA/ZL tipo S de 100 kg — dimensional do fabricante
(linha 100 kg): A=50,8 mm (largura), B=76,2 mm (altura), C=19,1 mm
(espessura), rosca M12×1,75 nos dois lados. Montagem AXIAL entre os
acopladores impressos (CAD Acoplador_CelulaDeCarga_Uniaxial.f3d, ⌀63,5 mm).
Os acopladores têm RECUOS que encaixam as faces superior/inferior da célula
e alojam o parafuso M12 — nada de rosca exposta entre as peças; as faces
assentam direto nos acopladores:

  Z (mm)   0 …   8   acoplador-robô (disco ⌀63,5), face inferior no flange
           8 …  84,2 corpo da célula S (50,8 × 19,1 × 76,2)
        84,2 …  92,2 acoplador-ferramenta (disco ⌀63,5)

Frame: origem no centro do acoplador-robô, face inferior em Z=0, tudo
coaxial a +Z (túnel do S visto de frente em Y). O touch_tool monta no topo
(Z=+92,2 mm) → TCP no probe em (0, 0, +206,7 mm) no frame do flange
(kinematics.T_TOUCH_TOOL_ATTACH espelha este valor).

Uso:  python3 gen_loadcell_100kg_assembly_stl.py   (grava em ../meshes/)
"""
import os

import trimesh

PLATE_R  = 31.75    # acopladores ⌀63,5 mm
PLATE_T  = 8.0

CELL_A = 50.8       # largura (X)
CELL_B = 76.2       # altura  (Z)
CELL_C = 19.1       # espessura (Y)
END_H  = 17.0       # blocos superior/inferior do S
MID_H  = 14.0       # barra central do S
WEB_W  = 13.0       # colunas laterais que fecham o S


def _box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=(x1 - x0, y1 - y0, z1 - z0))
    b.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    return b


def _cyl_z(cx, cy, z0, z1, r, sections=96):
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=sections)
    c.apply_translation((cx, cy, (z0 + z1) / 2))
    return c


def build() -> trimesh.Trimesh:
    hx, hy = CELL_A / 2, CELL_C / 2
    z_cell0 = PLATE_T                               # face inferior no recuo
    z_cell1 = z_cell0 + CELL_B

    gap = (CELL_B - 2 * END_H - MID_H) / 2          # vãos do S (≈14,1 mm)
    z_bot1 = z_cell0 + END_H                        # topo do bloco inferior
    z_mid0 = z_bot1 + gap
    z_mid1 = z_mid0 + MID_H
    z_top0 = z_mid1 + gap                           # base do bloco superior

    parts = [
        _cyl_z(0, 0, 0.0, PLATE_T, PLATE_R),                      # acoplador-robô
        _box(-hx, hx, -hy, hy, z_cell0, z_bot1),                  # bloco inferior
        _box(-hx, hx, -hy, hy, z_mid0, z_mid1),                   # barra central
        _box(-hx, hx, -hy, hy, z_top0, z_cell1),                  # bloco superior
        _box(-hx, -hx + WEB_W, -hy, hy, z_mid1, z_top0),          # coluna sup. (−X)
        _box(hx - WEB_W, hx, -hy, hy, z_bot1, z_mid0),            # coluna inf. (+X)
        _cyl_z(0, 0, z_cell1, z_cell1 + PLATE_T, PLATE_R),        # acoplador-tool
    ]

    return trimesh.util.concatenate(parts)


if __name__ == '__main__':
    mesh = build()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'meshes', 'CelulaDeCarga_100kg_Montagem.stl')
    out = os.path.normpath(out)
    mesh.export(out)
    print(f'{out}: {len(mesh.faces)} faces, bounds (mm):\n{mesh.bounds}')
