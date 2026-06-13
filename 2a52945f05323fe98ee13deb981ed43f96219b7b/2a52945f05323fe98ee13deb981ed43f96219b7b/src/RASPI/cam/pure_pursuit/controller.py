"""
Controlador geométrico Pure Pursuit operando en espacio de píxeles BEV.

Fórmula:
    alpha   = atan2(dx, dy)          # ángulo signed al punto look-ahead
                                     # + = derecha,  - = izquierda
    steer   = atan2(2·L·sin(alpha), lookahead_dist)   # en radianes
    steer°  = degrees(steer)         # acotado a ±MAX_STEER_DEG

Normalización para el protocolo serial V2:
    obs = steer° / PP_STEER_GAIN     # ∈ [-1, 1]
    ESP32 en modo pp=1 multiplica por ppSteerGain (35) para recuperar steer°.
"""

import math

from . import config as C


class PurePursuitController:
    """
    Controlador Pure Pursuit puro.
    Opera en coordenadas BEV donde el robot siempre apunta "hacia arriba"
    (Y decreciente = dirección de marcha).
    """

    def compute(
        self,
        path_points: list[tuple[int, int]],
        robot_x: int = C.ROBOT_BEV_X,
        robot_y: int = C.ROBOT_BEV_Y,
    ) -> tuple[float, tuple[float, float]]:
        """
        Calcula el ángulo de dirección y el punto look-ahead.

        Retorna:
          steer_deg    : ángulos en grados, + = derecha, - = izquierda
          lookahead_pt : (x, y) en coords BEV del punto objetivo
        """
        if not path_points:
            return 0.0, (float(robot_x), float(robot_y))

        # Busca el primer punto a distancia >= LOOKAHEAD_PX
        target = path_points[0]   # fallback: el más cercano disponible
        for pt in path_points:
            dist = math.hypot(pt[0] - robot_x, pt[1] - robot_y)
            if dist >= C.LOOKAHEAD_PX:
                target = pt
                break

        dx = target[0] - robot_x   # positivo = derecha
        dy = robot_y  - target[1]  # positivo = adelante (eje Y BEV invertido)
        ld = max(1.0, math.hypot(dx, dy))

        # alpha: ángulo al objetivo respecto al eje de marcha del robot
        alpha = math.atan2(dx, dy)   # + = derecha, - = izquierda

        steer_rad = math.atan2(2.0 * C.WHEELBASE_PX * math.sin(alpha), ld)
        steer_deg = math.degrees(steer_rad)
        steer_deg = max(-C.MAX_STEER_DEG, min(C.MAX_STEER_DEG, steer_deg))

        return steer_deg, (float(target[0]), float(target[1]))

    def normalize(self, steer_deg: float) -> float:
        """
        Normaliza steer_deg a [-1, 1] para el campo obs del protocolo V2.
        El ESP32 en modo pp=1 recupera steer_deg multiplicando por PP_STEER_GAIN.
        """
        return max(-1.0, min(1.0, steer_deg / C.PP_STEER_GAIN))
