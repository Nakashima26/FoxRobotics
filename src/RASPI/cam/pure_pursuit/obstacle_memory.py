"""
Memoria de obstáculos — mapa rodante DISPERSO en coordenadas BEV relativas al robot.

Problema que resuelve:
  El BEV se recalcula cada frame y solo contiene lo que la cámara ve AHORA.  Al
  acercarse a una lata, ésta se sale por abajo del cuadro, su inflación
  desaparece y la centerline brinca al centro → el carro se corta sobre la lata.

Idea (igual que el occupancy grid de simulacion_v2, pero sparse):
  Guardamos las latas vistas como una lista de (x, y, color, conf) en coords BEV.
  Cada frame las "arrastramos" según cuánto se movió el robot:
    - avance  : ds_px hacia abajo (la lata se acerca al robot)  → velocidad asumida
    - giro    : rotación −dθ alrededor del robot                → heading del IMU (ESP32)
  Luego fusionamos con las detecciones nuevas, decaemos la confianza de las que
  no se re-vieron y tiramos las que quedaron detrás del robot o ya expiraron.

  La lista resultante se pasa tal cual a detect_centerline() — así la inflación de
  la lata sigue presente hasta que el robot físicamente la rebasa.

Convención BEV (ver bev.py):
  X crece a la derecha, Y crece hacia ABAJO, robot en (ROBOT_BEV_X, ROBOT_BEV_Y)
  mirando hacia ARRIBA (Y decreciente = dirección de marcha).
"""

import math

from . import config as C


class _Obs:
    __slots__ = ("x", "y", "color", "conf")

    def __init__(self, x: float, y: float, color: str, conf: float):
        self.x = x
        self.y = y
        self.color = color
        self.conf = conf


class ObstacleMemory:
    """
    Mantiene el mapa rodante disperso.  Una instancia por runtime.

    Uso por frame:
        merged = mem.update(new_obs, dt_s, heading_deg)
        path   = detect_centerline(bev_frame, merged)
    """

    def __init__(self,
                 robot_x: int = C.ROBOT_BEV_X,
                 robot_y: int = C.ROBOT_BEV_Y):
        self.rx = robot_x
        self.ry = robot_y
        self._obs: list[_Obs] = []
        self._prev_heading: float | None = None

    def reset(self):
        self._obs.clear()
        self._prev_heading = None

    # ── Transformación por movimiento del robot ─────────────────────────────────

    def _advance(self, ds_px: float, dheading_deg: float):
        """
        Lleva cada obstáculo recordado del frame anterior al frame actual.

        El robot avanzó ds_px (hacia arriba/−Y) y giró dheading_deg.  En el marco
        relativo al robot eso equivale a: la lata baja ds_px y el mundo rota −dθ
        alrededor del robot.
        """
        # Rotación −dθ.  NOTA: si al doblar el mapa se desalinea hacia el lado
        # equivocado, invierte el signo aquí (depende de la orientación del gyro).
        phi = math.radians(-dheading_deg)
        cos_p, sin_p = math.cos(phi), math.sin(phi)

        for o in self._obs:
            # 1) avance: la lata se acerca → baja en la imagen
            uy = (o.y - self.ry) + ds_px
            ux = (o.x - self.rx)
            # 2) giro: rotar el vector relativo al robot
            rx_new = ux * cos_p - uy * sin_p
            ry_new = ux * sin_p + uy * cos_p
            o.x = self.rx + rx_new
            o.y = self.ry + ry_new

    # ── Fusión con detecciones nuevas ───────────────────────────────────────────

    def _merge(self, new_obs: list[tuple[float, float, str]]):
        match_r2 = C.OBS_MEM_MATCH_PX ** 2
        # Decaer todos primero; los que se re-vean recuperan confianza al fusionar.
        for o in self._obs:
            o.conf -= C.OBS_MEM_DECAY

        for nx, ny, color in new_obs:
            best = None
            best_d2 = match_r2
            for o in self._obs:
                if o.color != color:
                    continue
                d2 = (o.x - nx) ** 2 + (o.y - ny) ** 2
                if d2 <= best_d2:
                    best_d2 = d2
                    best = o
            if best is not None:
                # Re-visto: confiar en la posición fresca de la cámara.
                best.x, best.y = nx, ny
                best.conf = C.OBS_MEM_REFRESH
            else:
                if len(self._obs) < C.OBS_MEM_MAX:
                    self._obs.append(_Obs(nx, ny, color, C.OBS_MEM_REFRESH))

    # ── Poda ────────────────────────────────────────────────────────────────────

    def _prune(self):
        behind_y = self.ry + C.OBS_MEM_BEHIND_PAD
        kept: list[_Obs] = []
        for o in self._obs:
            if o.conf < C.OBS_MEM_MIN_CONF:
                continue
            if o.y > behind_y:                       # ya quedó detrás del robot
                continue
            if not (0.0 <= o.x < C.BEV_W and 0.0 <= o.y < C.BEV_H):
                continue
            kept.append(o)
        self._obs = kept

    # ── API ─────────────────────────────────────────────────────────────────────

    def update(self,
               new_obs: list[tuple[float, float, str]],
               dt_s: float,
               heading_deg: float | None
               ) -> list[tuple[float, float, str]]:
        """
        Avanza el mapa, fusiona detecciones nuevas y devuelve la lista combinada
        (x, y, color) lista para detect_centerline().

        new_obs     : obstáculos detectados este frame en coords BEV
        dt_s        : segundos transcurridos desde el update anterior
        heading_deg : ángulo del IMU (anguloGyro del ESP32) o None si no llegó
        """
        ds_px = (C.ROBOT_SPEED_MMS * dt_s) / C.MM_PER_PX if dt_s > 0 else 0.0

        if heading_deg is not None and self._prev_heading is not None:
            dheading = heading_deg - self._prev_heading
            dheading = (dheading + 180.0) % 360.0 - 180.0
        else:
            dheading = 0.0
        if heading_deg is not None:
            self._prev_heading = heading_deg

        self._advance(ds_px, dheading)
        self._merge(new_obs)
        self._prune()

        return [(o.x, o.y, o.color) for o in self._obs]
