"""lc_serial.py — Transporte SERIAL da célula de carga (XIAO ESP32S3, USB).

O firmware imprime cada amostra na USB CDC como linha

    F,<seq>,<t_us>,<v_sensor>

(mesmos campos do datagrama LOAD_CELL_SAMPLE_FMT '<IIf'), SEMPRE — em
paralelo ao UDP, porque "WiFi conectado" não prova que alguém recebe. Este
módulo lê a porta e entrega cada amostra a um callback; a palpation_gui liga
esse callback no mesmo pipeline do caminho UDP e deduplica (a serial é
ignorada enquanto o UDP está fresco).

A thread cuida de tudo sozinha (auto-detect por VID Espressif, abertura,
reabertura em hot-plug): start() uma vez, stop() no fechamento. Não depende
de ROS nem de Tkinter. Formato espelhado em sensors/ForceDriver/src/main.cpp.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

try:
    import serial
    from serial.tools import list_ports
    _SERIAL_OK = True
except Exception:  # pragma: no cover - pyserial ausente
    serial = None
    list_ports = None
    _SERIAL_OK = False

from .constants import (
    LOAD_CELL_SERIAL_BAUD,
    LOAD_CELL_SERIAL_PREFIX,
    LOAD_CELL_USB_VID,
)

# Intervalo entre tentativas de achar/abrir a porta (hot-plug).
_RETRY_S = 2.0


def detect_lc_serial_port() -> Optional[str]:
    """Porta USB do XIAO, achada pelo VID Espressif — o que a distingue do
    STM32 do touch sensor no mesmo PC (o detect do toque EXCLUI este VID)."""
    if not _SERIAL_OK:
        return None
    for p in list_ports.comports():
        if p.vid == LOAD_CELL_USB_VID:
            return p.device
    return None


class LoadCellSerialSource:
    """Leitor da linha "F,seq,t_us,v" do firmware em thread de fundo.

    ``on_sample(seq, t_us, v_sensor)`` é chamado NA THREAD DE LEITURA — o
    consumidor cuida da própria sincronização. ``connected``/``last_rx``
    diferenciam "porta aberta" de "dados chegando agora".
    """

    def __init__(self, port: Optional[str] = None,
                 on_sample: Optional[Callable[[int, int, float], None]] = None):
        # port=None → auto-detect por VID a cada tentativa (cobre replug com
        # renomeação do /dev/ttyACMx).
        self._port_req = port
        self._on_sample = on_sample
        self.port: Optional[str] = None
        self.connected = False
        # time.monotonic() da última amostra parseada (0.0 = nunca).
        self.last_rx: float = 0.0
        self.error: str = ''
        self._running = False
        self._ser = None
        self._thread: Optional[threading.Thread] = None

    # ──────────────────────────────────────────────────────────────────
    def start(self) -> bool:
        """Arma a thread. False só se pyserial não existe — a ausência do
        XIAO na USB não é falha: a thread fica tentando."""
        if not _SERIAL_OK:
            self.error = 'pyserial ausente'
            return False
        if self._running:
            return True
        self._running = True
        self._thread = threading.Thread(
            target=self._worker, daemon=True, name='lc-serial')
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False
        # Fecha a porta por fora para destravar o readline() da thread.
        ser = self._ser
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def send(self, data: bytes) -> bool:
        """Escreve um comando na porta (ex.: b'Z' = re-zero do firmware).
        False se a porta não está aberta agora. pyserial permite write de
        outra thread enquanto a de leitura está no readline()."""
        ser = self._ser
        if ser is None:
            return False
        try:
            ser.write(data)
            return True
        except Exception:
            return False

    # ──────────────────────────────────────────────────────────────────
    def _worker(self) -> None:
        while self._running:
            port = self._port_req or detect_lc_serial_port()
            if port is None:
                self.error = 'XIAO (VID 0x303A) ausente na USB'
                time.sleep(_RETRY_S)
                continue
            try:
                ser = serial.Serial(port, LOAD_CELL_SERIAL_BAUD, timeout=1.0)
            except Exception as exc:
                self.error = str(exc)
                time.sleep(_RETRY_S)
                continue
            self._ser = ser
            self.port = port
            self.connected = True
            self.error = ''
            try:
                self._read_loop(ser)
            except Exception as exc:
                # Desconexão (replug) ou porta fechada pelo stop().
                self.error = str(exc)
            finally:
                self.connected = False
                self._ser = None
                try:
                    ser.close()
                except Exception:
                    pass
            if self._running:
                time.sleep(_RETRY_S)

    def _read_loop(self, ser) -> None:
        prefix = LOAD_CELL_SERIAL_PREFIX
        while self._running:
            line = ser.readline()
            if not line:
                continue
            text = line.decode('ascii', 'ignore').strip()
            # Só linhas de amostra (ignora heartbeat '#' e lixo de boot).
            if not text.startswith(prefix):
                continue
            parts = text.split(',')
            if len(parts) != 4:
                continue
            try:
                seq = int(parts[1])
                t_us = int(parts[2])
                v_sensor = float(parts[3])
            except ValueError:
                continue
            self.last_rx = time.monotonic()
            if self._on_sample is not None:
                self._on_sample(seq, t_us, v_sensor)
