"""
Detección de líneas de esquina (naranja / azul) en la imagen BEV.

Recuperado del historial (commit 2ab26e2 ".ino updated + test turns",
2026-08-17, revertido al día siguiente junto con el intento de
segment_tracker.py) — la detección en sí identificaba bien las líneas;
lo que se revirtió fue la lógica de disparo de giro que se construyó
encima, no esto. Reintegrado con los umbrales HSV medidos en pista real
hoy (ver [HSV naranja] / [HSV banda debajo de naranja] en runtime_nuevo.py),
no los valores originales sin calibrar.

Convención BEV (ver bev.py): Y crece hacia ABAJO (hacia el robot). Un punto
con Y grande está CERCA del robot; Y chico está LEJOS (adelante).
"""

import cv2
import numpy as np

from . import config as C


def _line_mask(bev_hsv: np.ndarray, ranges) -> np.ndarray:
    masks = [cv2.inRange(bev_hsv, lo, hi) for lo, hi in ranges]
    return np.bitwise_or.reduce(masks) if len(masks) > 1 else masks[0]


def detect_lines(bev_bgr: np.ndarray) -> dict:
    """
    Retorna, por color: {'seen': bool, 'near_y': float|None}
    near_y = coordenada Y-BEV del punto MÁS CERCANO al robot de esa línea.
    """
    hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
    out = {}
    for color, ranges in (("Orange", C.LINE_ORANGE_HSV), ("Blue", C.LINE_BLUE_HSV)):
        mask = _line_mask(hsv, ranges)
        ys, _ = np.where(mask > 0)
        if len(ys) < C.LINE_MIN_PIXELS:
            out[color] = {"seen": False, "near_y": None}
        else:
            out[color] = {"seen": True, "near_y": float(ys.max())}
    return out
