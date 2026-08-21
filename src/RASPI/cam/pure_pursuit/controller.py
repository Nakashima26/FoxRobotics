"""
Controlador geométrico Pure Pursuit operando en espacio de píxeles BEV.
...
"""

import math

from . import config as C


class PurePursuitController:
    """
    Controlador Pure Pursuit puro.
    Opera en coordenadas BEV donde el robot siempre apunta "hacia arriba"
    (Y decreciente = dirección de marcha).
    """

    @staticmethod
    def adaptive_lookahead(
        bev_obstacles: list[tuple[float, float, str]],
        robot_x: int = C.ROBOT_BEV_X,
        robot_y: int = C.ROBOT_BEV_Y,
    ) -> float:
        """
        Calcula el lookahead efectivo según qué tan cerca está el obstáculo
        más próximo al robot.

        Sin obstáculos, o con el más cercano lejos  -> LOOKAHEAD_MAX_PX
        (trayectoria suave, estable en recta/curvas normales).

        Con un obstáculo a <= LOOKAHEAD_OBS_NEAR_PX  -> LOOKAHEAD_MIN_PX
        (apunta a un punto cercano del path -> geometría exige steer más
        cerrado -> giro fuerte para esquivar de inmediato).

        Entre ambos umbrales, interpola linealmente para que la transición
        no sea un salto brusco de steer.
        """
        if not bev_obstacles:
            return C.LOOKAHEAD_MAX_PX

        nearest_d = min(
            math.hypot(ox - robot_x, oy - robot_y) for ox, oy, _ in bev_obstacles
        )

        if nearest_d <= C.LOOKAHEAD_OBS_NEAR_PX:
            return C.LOOKAHEAD_MIN_PX
        if nearest_d >= C.LOOKAHEAD_OBS_FAR_PX:
            return C.LOOKAHEAD_MAX_PX

        t = (nearest_d - C.LOOKAHEAD_OBS_NEAR_PX) / (
            C.LOOKAHEAD_OBS_FAR_PX - C.LOOKAHEAD_OBS_NEAR_PX
        )
        return C.LOOKAHEAD_MIN_PX + t * (C.LOOKAHEAD_MAX_PX - C.LOOKAHEAD_MIN_PX)

    @staticmethod
    def _distance_steer_gain(bev_obstacles, robot_x, robot_y) -> float:
        if not bev_obstacles:
            return 1.0
        nearest_d = min(math.hypot(ox - robot_x, oy - robot_y) for ox, oy, _ in bev_obstacles)
        if nearest_d <= C.STEER_DIST_GAIN_NEAR_PX:
            return 1.0
        if nearest_d >= C.STEER_DIST_GAIN_FAR_PX:
            return C.STEER_DIST_GAIN_MIN
        t = (nearest_d - C.STEER_DIST_GAIN_NEAR_PX) / (C.STEER_DIST_GAIN_FAR_PX - C.STEER_DIST_GAIN_NEAR_PX)
        return 1.0 - t * (1.0 - C.STEER_DIST_GAIN_MIN)

    def compute(
        self,
        path_points: list[tuple[int, int]],
        robot_x: int = C.ROBOT_BEV_X,
        robot_y: int = C.ROBOT_BEV_Y,
        lookahead_px: float = C.LOOKAHEAD_PX,
        bev_obstacles: list[tuple[float, float, str]] | None = None,
    ) -> tuple[float, tuple[float, float]]:
        if not path_points:
            return 0.0, (float(robot_x), float(robot_y))

        if bev_obstacles:
            lo, hi = lookahead_px * 0.85, lookahead_px * 1.25
            candidates = [
                pt for pt in path_points
                if lo <= math.hypot(pt[0] - robot_x, pt[1] - robot_y) <= hi
            ]
            if candidates:
                target = max(candidates, key=lambda pt: abs(pt[0] - robot_x))
            else:
                target = path_points[-1]
                for pt in path_points:
                    if math.hypot(pt[0] - robot_x, pt[1] - robot_y) >= lookahead_px:
                        target = pt
                        break
        else:
            target = path_points[-1]
            for pt in path_points:
                dist = math.hypot(pt[0] - robot_x, pt[1] - robot_y)
                if dist >= lookahead_px:
                    target = pt
                    break

        dx = target[0] - robot_x
        dy = robot_y - target[1]
        ld = max(1.0, math.hypot(dx, dy))
        alpha = math.atan2(dx, dy)
        steer_rad = math.atan2(2.0 * C.WHEELBASE_PX * math.sin(alpha), ld)

        steer_deg = math.degrees(steer_rad)

        # ── NUEVO: atenúa el steer si el obstáculo más cercano aún está lejos ──
        if bev_obstacles:
            gain = self._distance_steer_gain(bev_obstacles, robot_x, robot_y)
            steer_deg *= gain

        steer_deg = max(-C.MAX_STEER_DEG, min(C.MAX_STEER_DEG, steer_deg))

        return steer_deg, (float(target[0]), float(target[1]))

    def normalize(self, steer_deg: float) -> float:
        return max(-1.0, min(1.0, steer_deg / C.PP_STEER_GAIN))