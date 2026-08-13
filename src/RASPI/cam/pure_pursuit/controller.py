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

Este módulo también incluye tres funciones de apoyo, pensadas para el caso
de obstáculos cerca y descentrados (donde el pipeline "normal" de centerline
puede fallar o quedarse corto):

  dynamic_lookahead()   — interpola el lookahead según la distancia al
                           obstáculo más cercano en memoria.
  emergency_avoid_steer()— steer directo por geometría cuando SÍ hay un
                           obstáculo en bev_obstacles pero detect_centerline()
                           no logró armar suficiente path.
  corner_avoid_steer()  — steer directo (más agresivo, menos preciso) para
                           objetos vistos cerca en cámara cruda que NO
                           proyectaron al BEV (típico: esquina extrema del
                           frame, fuera del área calibrada de la homografía).
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
        lookahead_px: float | None = None,
    ) -> tuple[float, tuple[float, float]]:
        """
        Calcula el ángulo de dirección y el punto look-ahead.

        lookahead_px : si se omite, usa C.LOOKAHEAD_FAR_PX (comportamiento
                       fijo de antes). Pásale el resultado de
                       dynamic_lookahead() para que se adapte a la distancia
                       del obstáculo más cercano.

        Retorna:
          steer_deg    : ángulos en grados, + = derecha, - = izquierda
          lookahead_pt : (x, y) en coords BEV del punto objetivo
        """
        if not path_points:
            return 0.0, (float(robot_x), float(robot_y))

        lk = lookahead_px if lookahead_px is not None else C.LOOKAHEAD_FAR_PX

        # Busca el primer punto a distancia >= lk
        target = path_points[-1]   # fallback: el más lejano disponible (path_points[0] está detrás del robot)
        for pt in path_points:
            dist = math.hypot(pt[0] - robot_x, pt[1] - robot_y)
            if dist >= lk:
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


# ─────────────────────────────────────────────────────────────────────────────
# Lookahead dinámico
# ─────────────────────────────────────────────────────────────────────────────

def dynamic_lookahead(
    bev_obstacles: list[tuple[float, float, str]],
    robot_x: int = C.ROBOT_BEV_X,
    robot_y: int = C.ROBOT_BEV_Y,
) -> float:
    """
    Lookahead adaptativo: corto si hay un obstáculo cerca (para que el
    controlador SÍ vea los puntos forzados de esquive, que quedan cerca del
    robot), largo si no hay nada cerca (para corregir suave hacia la pared).

    Interpola linealmente entre LOOKAHEAD_NEAR_PX y LOOKAHEAD_FAR_PX según la
    distancia al obstáculo recordado más cercano.
    """
    if not bev_obstacles:
        return C.LOOKAHEAD_FAR_PX

    nearest = min(
        math.hypot(ox - robot_x, oy - robot_y) for ox, oy, _ in bev_obstacles
    )

    lo, hi = C.OBS_LOOKAHEAD_MIN_DIST, C.OBS_LOOKAHEAD_MAX_DIST
    t = (nearest - lo) / max(1.0, (hi - lo))
    t = max(0.0, min(1.0, t))

    return C.LOOKAHEAD_NEAR_PX + t * (C.LOOKAHEAD_FAR_PX - C.LOOKAHEAD_NEAR_PX)


# ─────────────────────────────────────────────────────────────────────────────
# Esquive de emergencia — hay obstáculo en BEV pero el path falló
# ─────────────────────────────────────────────────────────────────────────────

def emergency_avoid_steer(
    bev_obstacles: list[tuple[float, float, str]],
    robot_x: int = C.ROBOT_BEV_X,
    robot_y: int = C.ROBOT_BEV_Y,
) -> float:
    """
    Steer de emergencia cuando detect_centerline() no logró armar suficiente
    path (MIN_PATH_PTS) pero SÍ hay un obstáculo conocido en bev_obstacles.
    No depende de segmentar piso libre: calcula directamente hacia qué lado
    virar según la regla de color WRO y la geometría del obstáculo más
    cercano, con el mismo margen de seguridad que usa detect_centerline()
    (OBS_INFLATE_R + OBS_BIAS_SHIFT).
    """
    if not bev_obstacles:
        return 0.0

    # El obstáculo más cercano al robot manda (el más urgente)
    ox, oy, color = min(
        bev_obstacles,
        key=lambda o: math.hypot(o[0] - robot_x, o[1] - robot_y),
    )

    dist = max(1.0, math.hypot(ox - robot_x, oy - robot_y))

    safe_shift = C.OBS_INFLATE_R + C.OBS_BIAS_SHIFT
    if color == "Red":
        target_x = ox + safe_shift    # pasar por la derecha del bloque
    else:  # Green
        target_x = ox - safe_shift    # pasar por la izquierda del bloque

    dx = target_x - robot_x
    dy = robot_y - oy
    alpha = math.atan2(dx, max(1.0, dy))

    steer_rad = math.atan2(2.0 * C.WHEELBASE_PX * math.sin(alpha), dist)
    steer_deg = math.degrees(steer_rad)

    return max(-C.MAX_STEER_DEG, min(C.MAX_STEER_DEG, steer_deg))


# ─────────────────────────────────────────────────────────────────────────────
# Esquive de "esquina" — objeto cerca en cámara cruda, sin proyección BEV
# ─────────────────────────────────────────────────────────────────────────────

def close_range_boost(
    steer_deg: float,
    bev_obstacles: list[tuple[float, float, str]],
    robot_x: int = C.ROBOT_BEV_X,
    robot_y: int = C.ROBOT_BEV_Y,
) -> float:
    """
    Refuerzo de seguridad para rango muy cercano (ver CLOSE_RANGE_BOOST_* en
    config.py). El steer que sale de pp_follow (basado en lookahead) puede
    salir tibio cuando el obstáculo está en la zona no calibrada de la
    homografía (<20cm), porque su distancia en BEV no refleja bien la
    distancia real. Aquí se compara contra emergency_avoid_steer() (geometría
    directa obstáculo→robot, no depende del lookahead) y se usa el que tenga
    mayor magnitud — siempre que ambos apunten en la misma dirección; si
    difieren en signo, se deja el steer original de pp_follow por seguridad
    (evita un cambio de dirección brusco por una lectura ruidosa).
    """
    if not bev_obstacles or not C.CLOSE_RANGE_BOOST_ENABLED:
        return steer_deg

    nearest = min(
        math.hypot(ox - robot_x, oy - robot_y) for ox, oy, _ in bev_obstacles
    )
    if nearest > C.CLOSE_RANGE_BOOST_DIST_PX:
        return steer_deg

    alt = emergency_avoid_steer(bev_obstacles, robot_x, robot_y)

    same_direction = (steer_deg == 0.0) or (alt == 0.0) or (
        (steer_deg > 0) == (alt > 0)
    )
    # Solo reemplaza si el steer de emergencia es CLARAMENTE mayor (+20%),
    # para no saltar de un valor a otro por diferencias chicas justo en el
    # borde de CLOSE_RANGE_BOOST_DIST_PX (eso se sentía como un giro brusco
    # de más).
    if same_direction and abs(alt) > abs(steer_deg) * 1.20:
        return alt
    return steer_deg


def corner_avoid_steer(
    close_unprojected: list[tuple[float, float, float, str]],
) -> float:
    """
    Esquive directo para obstáculos que la cámara ve claramente cerca (bbox
    abajo en el frame, ver CORNER_FOOT_Y_THRESHOLD en runtime) pero que NO
    proyectaron al BEV porque caen fuera del área que cubre la homografía
    calibrada — típicamente porque el objeto está en una esquina lateral
    extrema del frame, muy cerca del robot.

    No tenemos coordenadas BEV reales para este objeto, así que NO se puede
    usar la geometría precisa de emergency_avoid_steer().  En su lugar,
    aplicamos un steer fuerte (mitad de MAX_STEER_DEG como piso, escalando
    hasta el máximo según qué tan grande/cerca se ve el bbox) en la dirección
    que manda la regla de color WRO.  Es una respuesta "gruesa" pero segura
    mientras el objeto termina de entrar al rango calibrado del BEV (donde
    emergency_avoid_steer o el pipeline normal retoman con precisión).

    close_unprojected: lista de (center_x, w, h, color_str) — bboxes en
                        cámara cruda que fallaron map_obstacle_to_bev() y
                        tienen el pie (y+h) por debajo de
                        C.CORNER_FOOT_Y_THRESHOLD.
    """
    if not close_unprojected:
        return 0.0

    # El bbox más grande = el más cercano/urgente
    _cx, w, h, color = max(close_unprojected, key=lambda o: o[1] * o[2])
    sign = +1.0 if color == "Red" else -1.0

    area = w * h
    # Entre más grande el bbox (más cerca), más fuerte el steer.
    # área de referencia: 6x el área mínima que ya considera far_hint válida.
    urgency = min(1.0, area / (C.FAR_HINT_MIN_AREA_PX * 6.0))

    steer = sign * (C.MAX_STEER_DEG * 0.5 + urgency * C.MAX_STEER_DEG * 0.5)
    return max(-C.MAX_STEER_DEG, min(C.MAX_STEER_DEG, steer))
