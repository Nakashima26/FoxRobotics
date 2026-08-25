"""
Detección de líneas de esquina (naranja / azul) en la imagen BEV.

Recuperado del historial (commit 2ab26e2 ".ino updated + test turns",
2026-08-17), pero con un método de localización distinto: la versión
original tomaba el Y máximo de TODOS los pixeles del color en todo el
frame — un puñado de ruido disperso (que nunca forma una franja real)
contaba igual que la línea de verdad, y si el ruido caía más cerca del
robot que la línea real, ganaba el ruido.

Método actual: recorre el BEV fila por fila desde el robot hacia adelante
y se queda con la PRIMERA fila que tenga una corrida CONTIGUA de ese color
de al menos LINE_MIN_RUN_PX — exige que sea una franja real en esa fila
(no puntos sueltos), y como se evalúa fila por fila (no un bounding box de
todo el blob), una línea curva o en diagonal no se penaliza por su altura
total como sí le pasaba al enfoque de bounding box.

Convención BEV (ver bev.py): Y crece hacia ABAJO (hacia el robot). Un punto
con Y grande está CERCA del robot; Y chico está LEJOS (adelante).
"""

import cv2
import numpy as np

from . import config as C


def _line_mask(bev_hsv: np.ndarray, ranges) -> np.ndarray:
    masks = [cv2.inRange(bev_hsv, lo, hi) for lo, hi in ranges]
    return np.bitwise_or.reduce(masks) if len(masks) > 1 else masks[0]


def _longest_run(row: np.ndarray) -> int:
    """Ancho de la corrida contigua de pixeles>0 más larga en una fila 1D."""
    cols = np.where(row > 0)[0]
    if len(cols) == 0:
        return 0
    breaks = np.where(np.diff(cols) > 1)[0]
    runs = np.split(cols, breaks + 1)
    return max(len(r) for r in runs)


def _find_near_line_row(mask: np.ndarray, min_run_px: int) -> float | None:
    """
    Escanea desde la fila más cercana al robot (Y grande) hacia adelante
    (Y chico) y retorna la Y de la primera fila con una corrida contigua
    >= min_run_px. None si ninguna fila califica.
    """
    h = mask.shape[0]
    for y in range(h - 1, -1, -1):
        if _longest_run(mask[y, :]) >= min_run_px:
            return float(y)
    return None


def detect_lines(bev_bgr: np.ndarray) -> dict:
    """
    Retorna {'seen': bool, 'near_y': float|None} para la línea naranja.
    near_y = coordenada Y-BEV de la franja real más cercana al robot
    (no el pixel aislado más cercano — ver _find_near_line_row()).

    Azul se quitó de la ecuación (daba muchos falsos positivos/negativos y
    no era confiable) — por ahora solo se seguirá la línea naranja.

    Lectura CRUDA de un solo frame — brinca entre un segmento cercano
    parcialmente ocluido por un obstáculo (a veces cruza LINE_MIN_RUN_PX,
    a veces no, por un par de pixeles de diferencia frame a frame) y el
    siguiente segmento realmente visible más lejos. Para uso en el
    runtime real, usar OrangeLineTracker (abajo), que suaviza esto en
    el tiempo.
    """
    hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
    mask = _line_mask(hsv, C.LINE_ORANGE_HSV)
    near_y = _find_near_line_row(mask, C.LINE_MIN_RUN_PX)
    return {"Orange": {"seen": near_y is not None, "near_y": near_y}}


class OrangeLineTracker:
    """
    Suaviza detect_lines() en el tiempo.

    La lectura cruda de un solo frame puede "brincar" entre dos segmentos
    reales distintos (uno cercano parcialmente ocluido por un obstáculo,
    otro más lejos y despejado) de un frame a otro — confirmado en pista:
    la Y reportada saltaba entre "muy pegada al obstáculo" y "bien
    separada" sin que el robot se moviera lo suficiente para justificarlo.

    En vez de confiar en la lectura de un solo frame, exige que una nueva
    lectura (mismo 'seen' y near_y dentro de una tolerancia) se repita
    PERSIST_FRAMES seguidos antes de aceptarla como el estado "real" — un
    brinco de un solo frame no alcanza a mover el valor reportado.
    """

    def __init__(self, persist_frames: int = 3, tolerance_px: float = 15.0):
        self.persist_frames = persist_frames
        self.tolerance_px = tolerance_px
        self.stable: dict = {"seen": False, "near_y": None}
        self._candidate: dict | None = None
        self._candidate_count = 0

    def reset(self):
        self.stable = {"seen": False, "near_y": None}
        self._candidate = None
        self._candidate_count = 0

    def _matches_candidate(self, raw: dict) -> bool:
        if self._candidate is None or raw["seen"] != self._candidate["seen"]:
            return False
        if not raw["seen"]:
            return True
        return abs(raw["near_y"] - self._candidate["near_y"]) <= self.tolerance_px

    def update(self, bev_bgr: np.ndarray) -> dict:
        raw = detect_lines(bev_bgr)["Orange"]

        if self._matches_candidate(raw):
            self._candidate_count += 1
        else:
            self._candidate = raw
            self._candidate_count = 1

        if self._candidate_count >= self.persist_frames:
            self.stable = self._candidate

        return self.stable
