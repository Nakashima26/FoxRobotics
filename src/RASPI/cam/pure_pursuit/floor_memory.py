"""
Memoria de piso — occupancy rodante en coordenadas BEV relativas al robot.

Problema que resuelve:
  El BEV se recalcula cada frame y la centerline solo ve la cuña de homografía
  de AHORA.  Al girar, una orilla se sale del triángulo, la máscara HSV pierde
  un lado y la línea brinca o se corta.

Idea (misma cinemática que obstacle_memory.py, pero sobre un grid 400×400):
  Guardamos una confianza 0..1 de "esta celda es piso".  Cada frame:
    1. Arrastramos el grid con avance asumido + giro del IMU.
    2. Donde la homografía SÍ observó, mezclamos con la máscara HSV actual
       (más peso cerca del robot, menos arriba donde el warp miente).
    3. Donde NO observó (negro de la cuña), solo decae el recuerdo.
  detect_centerline() recibe la máscara fusionada en vez del HSV crudo.

Convención BEV: igual que obstacle_memory.py / bev.py.
"""

import math

import cv2
import numpy as np

from . import config as C


class FloorMemory:
    """Una instancia por runtime.  Uso por frame:

        fused = floors.update(floor_mask, valid_mask, dt_s, heading_deg)
        path  = detect_centerline(bev_frame, obstacles, floor_mask=fused)
    """

    def __init__(self,
                 robot_x: int = C.ROBOT_BEV_X,
                 robot_y: int = C.ROBOT_BEV_Y):
        self.rx = robot_x
        self.ry = robot_y
        self.conf = np.zeros((C.BEV_H, C.BEV_W), dtype=np.float32)
        self._has = False
        self._prev_heading: float | None = None

        ys = np.arange(C.BEV_H, dtype=np.float32)
        # t=0 junto al robot (abajo), t=1 arriba / lejos
        t = np.clip((self.ry - ys) / max(float(self.ry), 1.0), 0.0, 1.0)
        w_row = C.FLOOR_MEM_NEAR_W * (1.0 - t) + C.FLOOR_MEM_FAR_W * t
        self._w = np.repeat(w_row[:, None], C.BEV_W, axis=1).astype(np.float32)

        k3 = np.ones((3, 3), np.uint8)
        k5 = np.ones((5, 5), np.uint8)
        self._k_open = k3
        self._k_close = k5

    def reset(self):
        self.conf.fill(0.0)
        self._has = False
        self._prev_heading = None

    # ── Warp = mismo _advance que las latas, aplicado a una imagen ────────────

    def _warp(self, grid: np.ndarray, ds_px: float, dheading_deg: float) -> np.ndarray:
        """
        Lleva el grid del frame anterior al actual.

        Inversa de obstacle_memory._advance: para cada píxel destino (nuevo)
        muestreamos el origen en el grid viejo.  dheading=0, ds>0 → el piso
        recordado baja hacia el robot (el mundo se acerca).
        """
        theta = math.radians(dheading_deg)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        rx, ry = float(self.rx), float(self.ry)

        M = np.array([
            [cos_t, -sin_t, -rx * cos_t + ry * sin_t + rx],
            [sin_t,  cos_t, -rx * sin_t - ry * cos_t - ds_px + ry],
        ], dtype=np.float32)

        return cv2.warpAffine(
            grid, M, (C.BEV_W, C.BEV_H),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0.0,
        )

    def update(
        self,
        floor_mask: np.ndarray,
        valid_mask: np.ndarray,
        dt_s: float,
        heading_deg: float | None,
    ) -> np.ndarray:
        """
        Avanza el mapa, fusiona la observación actual y devuelve máscara
        uint8 0/255 lista para detect_centerline(..., floor_mask=...).

        floor_mask  : HSV del frame actual (tras OPEN/CLOSE), 0 o 255
        valid_mask  : True donde la homografía cubre cámara (no el negro)
        dt_s        : segundos desde el update anterior
        heading_deg : anguloGyro del ESP32, o None si aún no llegó ACK
        """
        dt_s = min(max(float(dt_s), 0.0), 0.20)
        ds_px = (C.ROBOT_SPEED_MMS * dt_s) / C.MM_PER_PX if dt_s > 0 else 0.0

        if heading_deg is not None and self._prev_heading is not None:
            dheading = heading_deg - self._prev_heading
            dheading = (dheading + 180.0) % 360.0 - 180.0
        else:
            dheading = 0.0
        if heading_deg is not None:
            self._prev_heading = heading_deg

        current = (floor_mask > 0).astype(np.float32)
        valid = valid_mask.astype(bool)

        if not self._has:
            self.conf[valid] = current[valid]
            self._has = True
            return self._to_mask(self.conf)

        warped = self._warp(self.conf, ds_px, dheading)

        tau = max(C.FLOOR_MEM_TAU_S, 1e-3)
        decay = math.exp(-dt_s / tau) if dt_s > 0 else 1.0

        fused = warped.copy()
        fused[~valid] *= decay
        w = self._w
        fused[valid] = warped[valid] * (1.0 - w[valid]) + current[valid] * w[valid]
        np.clip(fused, 0.0, 1.0, out=fused)
        self.conf = fused

        return self._to_mask(fused)

    def _to_mask(self, conf: np.ndarray) -> np.ndarray:
        binary = (conf >= C.FLOOR_MEM_THRESH).astype(np.uint8) * 255
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  self._k_open)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self._k_close)
        return binary
