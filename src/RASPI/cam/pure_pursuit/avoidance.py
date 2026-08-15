"""
Esquiva de obstáculos por cinemática inversa (tangente al círculo de exclusión).

En vez de sesgar la máscara de piso para "empujar" la centerline lejos del
obstáculo (método anterior: OBS_BIAS_SHIFT, dependiente del decay de memoria
e impreciso), calculamos directamente el punto tangente al círculo de
seguridad alrededor del obstáculo y ese punto se usa como target de Pure
Pursuit. Se recalcula CADA FRAME -> no hay acumulación de error, no depende
de tiempo ni de velocidad asumida (a diferencia de obstacle_memory.py, que sí
la necesita solo para el dead-reckoning de posición, no para el steer).
"""

import math

from . import config as C


def compute_tangent_target(robot_x, robot_y, obs_x, obs_y, radius_px, prefer_side):
    """
    Calcula el punto tangente al círculo de radio `radius_px` centrado en el
    obstáculo, del lado `prefer_side` ("right" o "left").

    Retorna (tx, ty), o None si el robot ya está DENTRO del radio de
    seguridad (no existe tangente real -> caso de emergencia).
    """
    dx, dy = obs_x - robot_x, obs_y - robot_y
    d = math.hypot(dx, dy)
    if d <= radius_px:
        return None

    tangent_len = math.sqrt(d * d - radius_px * radius_px)
    theta = math.asin(radius_px / d)
    base_angle = math.atan2(dy, dx)

    a1, a2 = base_angle + theta, base_angle - theta
    t1 = (robot_x + tangent_len * math.cos(a1), robot_y + tangent_len * math.sin(a1))
    t2 = (robot_x + tangent_len * math.cos(a2), robot_y + tangent_len * math.sin(a2))

    # Convención BEV: X crece hacia la derecha.
    if prefer_side == "right":
        return t1 if t1[0] > t2[0] else t2
    else:
        return t1 if t1[0] < t2[0] else t2


def select_active_obstacle(bev_obstacles, robot_x, robot_y, action_range_px):
    """
    Escoge el obstáculo más relevante para esquivar AHORA: el más cercano al
    robot, dentro del rango de acción, y que todavía esté "adelante"
    (oy <= robot_y; en BEV, Y crece hacia abajo/atrás).

    Retorna (ox, oy, color) o None si no hay ninguno relevante.
    """
    candidates = []
    for ox, oy, color in bev_obstacles:
        if oy > robot_y:
            continue   # ya quedó detrás del robot
        dist = math.hypot(ox - robot_x, oy - robot_y)
        if dist <= action_range_px:
            candidates.append((dist, ox, oy, color))

    if not candidates:
        return None
    candidates.sort(key=lambda c: c[0])
    _, ox, oy, color = candidates[0]
    return ox, oy, color


def side_for_color(color):
    # Regla WRO: Rojo -> pasar por la DERECHA, Verde -> pasar por la IZQUIERDA
    return "right" if color == "Red" else "left"


class AvoidanceController:
    """
    Envuelve el cálculo de tangente + estado de transición suave de vuelta a
    centerline. Una instancia por runtime.
    """

    def __init__(self):
        self._blend_remaining = 0
        self._last_avoid_steer = 0.0

    @property
    def blend_remaining(self) -> int:
        return self._blend_remaining

    def compute(self, bev_obstacles, robot_x, robot_y, pp_controller):
        """
        Retorna (steer_deg, lookahead_pt, active, blending):
          active   : True si hay un obstáculo relevante esquivándose AHORA
          blending : True si estamos en transición de vuelta a centerline
                     (el llamador debe mezclar con el steer de centerline)
        """
        target_obs = select_active_obstacle(
            bev_obstacles, robot_x, robot_y, C.AVOID_ACTION_RANGE_PX
        )

        if target_obs is None:
            if self._blend_remaining > 0:
                self._blend_remaining -= 1
                return self._last_avoid_steer, (float(robot_x), float(robot_y)), False, True
            return 0.0, (float(robot_x), float(robot_y)), False, False

        ox, oy, color = target_obs
        side = side_for_color(color)
        target = compute_tangent_target(robot_x, robot_y, ox, oy, C.AVOID_RADIUS_PX, side)

        if target is None:
            # Emergencia: ya dentro del radio de seguridad -> steer máximo al lado correcto
            steer_deg = C.MAX_STEER_DEG if side == "right" else -C.MAX_STEER_DEG
            lookahead_pt = (float(robot_x), float(robot_y))
        else:
            steer_deg, lookahead_pt = pp_controller.compute([target], robot_x, robot_y)

        self._last_avoid_steer = steer_deg
        self._blend_remaining = C.AVOID_BLEND_FRAMES
        return steer_deg, lookahead_pt, True, False

    def reset(self):
        self._blend_remaining = 0
        self._last_avoid_steer = 0.0