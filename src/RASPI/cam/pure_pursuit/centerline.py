"""
Detección de centerline en imagen BEV y mapeo de obstáculos a coordenadas BEV.

Algoritmo:
  1. Máscara de piso (HSV FLOOR_LOWER / FLOOR_UPPER) en la imagen BEV.
  2. Elimina zonas de obstáculos (inflado simétrico + sesgo asimétrico WRO).
  3. Muestrea filas de abajo hacia arriba. En cada fila mezcla el centro del
     hueco libre con el lado de paso WRO, con peso que rampa al acercarse a
     la lata (no un switch binario en Y).
  4. Media móvil 1-D en X + tope de Δx por paso (curvatura de servo).

Sesgo de color WRO:
  Rojo  → el robot pasa por la DERECHA del bloque
           → se bloquea más a la IZQUIERDA del bloque en BEV
  Verde → el robot pasa por la IZQUIERDA del bloque
           → se bloquea más a la DERECHA del bloque en BEV
"""

import math

import cv2
import numpy as np

from .bev import BEVTransformer
from . import config as C


# ── Mapeo de obstáculos al plano BEV ──────────────────────────────────────────

def map_obstacle_to_bev(
    bev: BEVTransformer,
    cam_x: float, cam_y: float,
    cam_w: float, cam_h: float,
) -> tuple[float, float] | None:
    """
    Proyecta el punto de contacto con el suelo del obstáculo (centro-inferior
    del bounding box) desde coordenadas de cámara a coordenadas BEV.

    Retorna (bev_x, bev_y) si el punto cae dentro de la imagen BEV, o None.
    """
    foot_x = cam_x + cam_w * 0.5
    foot_y = cam_y + cam_h        # fondo del bbox ≈ suelo
    result = bev.cam_to_bev(foot_x, foot_y)
    if result is None:
        return None
    bx, by = result
    if bev.bev_in_bounds(bx, by):
        return bx, by
    return None


# ── Helpers internos ──────────────────────────────────────────────────────────

def _widest_free_segment_bounds(row_mask: np.ndarray, min_width: int) -> tuple[int, int] | None:
    """
    Retorna (izq, der) del segmento libre más ancho en una fila de máscara.
    Expone los bordes (no solo el centro) para poder ajustar la tendencia del
    borde de la pared a lo largo de varias filas — ver `detect_centerline()`.
    """
    free_cols = np.where(row_mask > 0)[0]
    if len(free_cols) < min_width:
        return None

    best_l = best_r = -1
    best_w = 0
    seg_l = seg_prev = int(free_cols[0])

    for c in free_cols[1:]:
        ci = int(c)
        if ci > seg_prev + 1:
            w = seg_prev - seg_l + 1
            if w > best_w:
                best_w, best_l, best_r = w, seg_l, seg_prev
            seg_l = ci
        seg_prev = ci

    w = seg_prev - seg_l + 1
    if w > best_w:
        best_w, best_l, best_r = w, seg_l, seg_prev

    if best_w < min_width:
        return None
    return best_l, best_r


def _widest_free_segment(row_mask: np.ndarray, min_width: int) -> int | None:
    """Retorna el centro-x del segmento libre más ancho en una fila de máscara."""
    bounds = _widest_free_segment_bounds(row_mask, min_width)
    if bounds is None:
        return None
    return (bounds[0] + bounds[1]) // 2


def _ramp_weight(row_y: int, obs_y: float) -> float:
    """
    0 = ignorar la lata (usar centro del pasillo), 1 = lado de paso completo.
    La rampa entra al acercarse a la lata y se mantiene en 1 en el inflado;
    al rebasarla decae en un radio de inflación para volver al centro.
    """
    full_r = float(C.OBS_INFLATE_R)
    ramp = max(full_r + 1.0, float(C.CENTERLINE_RAMP_PX))
    d = float(row_y) - float(obs_y)  # >0: la fila aún no llega a la lata

    if d >= ramp:
        return 0.0
    if d > full_r:
        return 1.0 - (d - full_r) / (ramp - full_r)
    if d >= -full_r:
        return 1.0
    return max(0.0, 1.0 + (d + full_r) / full_r)


def _pass_side_cx(row_mask: np.ndarray, safe_row_mask: np.ndarray, ox: float, color: str) -> int | None:
    """Centro del hueco libre del lado WRO correcto en esta fila, priorizando piso a distancia segura de la pared."""
    iox = int(ox)
    iox = max(0, min(iox, row_mask.shape[0] - 1))
    pref_min = max(1, C.CENTERLINE_MIN_WIDTH // 2)

    if color == "Red":
        cx_rel = _widest_free_segment(safe_row_mask[iox:], pref_min)
        if cx_rel is None:
            cx_rel = _widest_free_segment(row_mask[iox:], pref_min)  # fallback sin margen
        return (iox + cx_rel) if cx_rel is not None else iox + C.OBS_PHYSICAL_R_PX

    if color == "Green":
        cx_abs = _widest_free_segment(safe_row_mask[:iox], pref_min)
        if cx_abs is None:
            cx_abs = _widest_free_segment(row_mask[:iox], pref_min)
        return cx_abs if cx_abs is not None else iox - C.OBS_PHYSICAL_R_PX

    return None


def _wall_urgency_cx(row_mask: np.ndarray, side: int) -> int | None:
    """
    Centro del hueco libre más ancho al lado del chasis indicado por `side`
    (-1 = izquierda, +1 = derecha de ROBOT_BEV_X), usado cuando el chasis
    apunta de frente a una pared y hay que abrirse hacia el lado despejado
    sin importar el sesgo de color WRO (aquí no hay tiempo para eso).
    """
    mid = C.ROBOT_BEV_X
    pref_min = max(1, C.CENTERLINE_MIN_WIDTH // 2)
    if side < 0:
        return _widest_free_segment(row_mask[:mid], pref_min)
    seg = _widest_free_segment(row_mask[mid:], pref_min)
    return (mid + seg) if seg is not None else None


def _smooth_x(points, weights=None):
    n = len(points)
    win = int(C.CENTERLINE_SMOOTH_WIN)
    if n < 3 or win < 3:
        return points
    if win % 2 == 0:
        win += 1
    xs = np.array([p[0] for p in points], dtype=np.float64)
    kernel = np.ones(win) / win
    pad = win // 2
    padded = np.pad(xs, pad, mode="edge")
    xs_s = np.convolve(padded, kernel, mode="valid")

    if weights is not None:
        # Blend: en peso alto confía más en el valor crudo, pero sin
        # abandonar el suavizado del todo (evita que ruido puntual dispare el steer).
        for i, wgt in enumerate(weights):
            blend = min(1.0, wgt) * 0.7   # máximo 70% crudo, nunca 100%
            xs_s[i] = (1.0 - blend) * xs_s[i] + blend * xs[i]

    return [(float(xs_s[i]), points[i][1]) for i in range(n)]


def _limit_lateral_step(points: list[tuple[float, int]], weights: list[float] | None = None) -> list[tuple[float, int]]:
    """
    Acota |Δx| entre filas consecutivas a lo que el servo puede.
    El cap se RELAJA proporcionalmente al peso de esquiva (weight): en zona
    libre protege contra ruido; en zona de esquiva activa permite alcanzar
    el offset completo rápido, porque ahí la urgencia real manda y
    controller.compute() ya aplica el límite físico real vía geometría.
    """
    if len(points) < 2:
        return points
    base_max_dx = math.tan(math.radians(C.MAX_STEER_DEG)) * float(C.CENTERLINE_ROW_STEP)
    relax = float(getattr(C, "CENTERLINE_URGENCY_RELAX", 3.0))

    out: list[tuple[float, int]] = [points[0]]
    prev_x = float(points[0][0])
    for i in range(1, len(points)):
        x, y = points[i]
        wgt = weights[i] if weights is not None else 0.0
        max_dx = base_max_dx * (1.0 + wgt * (relax - 1.0))
        dx = x - prev_x
        if dx > max_dx:
            x = prev_x + max_dx
        elif dx < -max_dx:
            x = prev_x - max_dx
        out.append((x, y))
        prev_x = x
    return out


def _extract_thin_blue_line(hsv: np.ndarray, shape: tuple) -> np.ndarray:
    """
    Separa la cinta azul delgada de la pared azul/gris gruesa dentro del
    candidato de color 'ancho'.

    Estrategia:
      1. Umbral ancho de azul (FLOOR_LOWER_BLUE_WIDE/UPPER) — atrapa tanto la
         cinta lejana (se ve grisácea) como la pared.
      2. Apertura morfológica con kernel ~WALL_STRUCTURE_PX: solo sobreviven
         blobs más gruesos que el kernel → eso es pared.
      3. Blobs que tocan el borde de la imagen BEV se tratan también como
         pared/fuera de pista (la cinta guía siempre está dentro del área).
      4. Lo que resta del candidato ancho, quitando lo anterior (dilatado un
         poco para limpiar bordes), es la línea delgada real.
    """
    h, w = shape[:2]
    blue_wide = cv2.inRange(hsv, C.FLOOR_LOWER_BLUE_WIDE, C.FLOOR_UPPER_BLUE_WIDE)

    if cv2.countNonZero(blue_wide) == 0:
        return blue_wide

    k_wall = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (C.WALL_STRUCTURE_PX, C.WALL_STRUCTURE_PX)
    )

    # Blobs gruesos que sobreviven a la apertura = pared.
    wall_blobs = cv2.morphologyEx(blue_wide, cv2.MORPH_OPEN, k_wall)

    # Componentes conectados del candidato ancho: descarta también los que
    # tocan el borde de la imagen BEV (casi siempre pared, nunca la cinta
    # guía que vive dentro del área jugable).
    n, labels, stats, _ = cv2.connectedComponentsWithStats(blue_wide, connectivity=8)
    border_blobs = np.zeros_like(blue_wide)
    for i in range(1, n):
        x, y, bw, bh, _area = stats[i]
        touches_border = (x <= 0 or y <= 0 or (x + bw) >= w or (y + bh) >= h)
        if touches_border:
            border_blobs[labels == i] = 255

    # Unir pared detectada por grosor + pared detectada por tocar el borde,
    # dilatar un poco para no dejar un halo de línea pegado a la pared.
    not_line = cv2.bitwise_or(wall_blobs, border_blobs)
    not_line = cv2.dilate(not_line, k_wall)

    thin_blue_line = cv2.bitwise_and(blue_wide, cv2.bitwise_not(not_line))
    return thin_blue_line

# ── Detección de centerline ───────────────────────────────────────────────────

def detect_centerline(
    bev_bgr: np.ndarray,
    bev_obstacles: list[tuple[float, float, str]],
) -> list[tuple[int, int]]:
    """
    Detecta la línea central del corredor en la imagen BEV.

    Con obstáculos de color, el X de cada fila es una mezcla entre el centro
    del hueco libre y el lado de paso WRO. El peso rampa con la distancia
    longitudinal a la lata (CENTERLINE_RAMP_PX), después se filtra X y se
    acota el Δx por paso a tan(MAX_STEER)·ROW_STEP.

    Parámetros:
      bev_bgr      : imagen BEV en BGR (BEV_W × BEV_H)
      bev_obstacles: lista de (bev_x, bev_y, color_str)  color_str = "Red"|"Green"

    Retorna lista de (x, y) en coordenadas BEV, ordenada de abajo (robot)
    hacia arriba (adelante).  Lista vacía si no hay suficiente piso visible.
    """
    h, w = bev_bgr.shape[:2]

    # ── 1. Máscara de piso ────────────────────────────────────────────────────
    hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)

    # 1a. Piso "seguro": beige/naranja/azul estricto — nunca confunde pared.
    floor_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in C.FLOOR_COLOR_RANGES:
        floor_mask |= cv2.inRange(hsv, lower, upper)

    # 1b. Línea azul delgada filtrada por FORMA (no solo color): un candidato
    # de color azul "ancho" (atrapa línea lejana grisácea + pared) se separa
    # en blobs gruesos (pared) vs delgados (cinta) usando apertura morfológica.
    thin_blue_line = _extract_thin_blue_line(hsv, bev_bgr.shape)
    floor_mask |= thin_blue_line

    k3 = np.ones((3, 3), np.uint8)
    k5 = np.ones((5, 5), np.uint8)
    floor_mask = cv2.morphologyEx(floor_mask, cv2.MORPH_OPEN,  k3)
    floor_mask = cv2.morphologyEx(floor_mask, cv2.MORPH_CLOSE, k5)

    # ── 2. Eliminar obstáculos + sesgo de color WRO ───────────────────────────
    free_mask = floor_mask.copy()
    for ox, oy, color in bev_obstacles:
        ix, iy = int(round(ox)), int(round(oy))

        # Zona de seguridad simétrica alrededor del obstáculo
        cv2.circle(free_mask, (ix, iy), C.OBS_INFLATE_R, 0, -1)

        # Sesgo asimétrico según reglas WRO de color
        if color == "Red":
            # Robot debe pasar por la DERECHA → bloquear más a la izquierda del bloque
            cx_bias = ix - C.OBS_BIAS_SHIFT
            cv2.circle(free_mask, (cx_bias, iy), C.OBS_INFLATE_R, 0, -1)
        elif color == "Green":
            # Robot debe pasar por la IZQUIERDA → bloquear más a la derecha del bloque
            cx_bias = ix + C.OBS_BIAS_SHIFT
            cv2.circle(free_mask, (cx_bias, iy), C.OBS_INFLATE_R, 0, -1)

    # ── 2b. Margen de seguridad contra CUALQUIER borde no-piso (pared incl.) ──
    # distanceTransform da, por pixel libre, la distancia al pixel no-libre
    # más cercano (pared, obstáculo, borde de imagen fuera del piso, etc.).
    # safe_mask solo deja pixeles a >= WALL_MARGIN_PX de cualquier borde así,
    # evitando que el centerline se pegue a la pared cuando el robot la mira
    # de frente y solo ve un triángulo de piso pegado a la esquina.
    dist = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)
    safe_mask = np.where(dist >= C.WALL_MARGIN_PX, free_mask, 0).astype(np.uint8)

    # ── 2c. Urgencia frontal por pared: ¿el chasis mismo apunta a una pared? ──
    # Se revisa un carril angosto centrado en el eje del chasis (ROBOT_BEV_X),
    # NO el hueco que el centerline elegiría — así se distingue "voy de frente
    # contra la pared" (carril frontal bloqueado) de "hay pared cerca a un
    # lado mientras avanzo recto" (carril frontal libre, corredor angosto
    # normal), que no debe disparar esta corrección.
    hw = C.FRONT_CHECK_HALFWIDTH_PX
    x_lo = max(0, C.ROBOT_BEV_X - hw)
    x_hi = min(w, C.ROBOT_BEV_X + hw)
    y_front_top = max(0, C.ROBOT_BEV_Y - int(C.FRONT_WALL_CRITICAL_PX))
    front_band = free_mask[y_front_top:C.ROBOT_BEV_Y, x_lo:x_hi]
    front_blocked = front_band.size > 0 and bool(np.any(front_band == 0))

    wall_side = 0
    if front_blocked:
        # ¿de qué lado hay más piso libre a la altura crítica? Ese es hacia
        # donde se fuerza el path — con toda su urgencia, sin rampa: es una
        # condición de "voy a chocar", no una esquiva planeada.
        y_check = max(C.CENTERLINE_TOP_Y, y_front_top)
        row_check = free_mask[y_check, :]
        left_free  = int(np.count_nonzero(row_check[:C.ROBOT_BEV_X]))
        right_free = int(np.count_nonzero(row_check[C.ROBOT_BEV_X:]))
        wall_side = -1 if left_free >= right_free else 1

    # ── 3. Muestreo fila a fila ────────────────────────────────────────────
    points: list[tuple[float, int]] = []
    weights: list[float] = []
    # Historial de bordes (y, izq, der) del hueco safe_mask en filas con dato
    # real -- permite, cuando una fila más adelante se queda sin piso, AJUSTAR
    # la tendencia del borde que se está cerrando y proyectarla hacia
    # adelante, en vez de reaccionar solo a lo que ya es visible (ver rama
    # "sin piso" más abajo).
    edge_hist: list[tuple[int, int, int]] = []
    for y in range(h - 10, C.CENTERLINE_TOP_Y, -C.CENTERLINE_ROW_STEP):
        row = free_mask[y, :]

        safe_bounds = _widest_free_segment_bounds(safe_mask[y, :], C.CENTERLINE_MIN_WIDTH)
        if safe_bounds is not None:
            edge_hist.append((y, safe_bounds[0], safe_bounds[1]))
            free_cx = (safe_bounds[0] + safe_bounds[1]) // 2
        else:
            free_cx = _widest_free_segment(row, C.CENTERLINE_MIN_WIDTH)

        best_w = 0.0
        pass_cx: int | None = None
        for ox, oy, color in bev_obstacles:
            if color not in ("Red", "Green"):
                continue
            wgt = _ramp_weight(y, oy)
            if wgt > best_w:
                cx_side = _pass_side_cx(row, safe_mask[y, :], ox, color)
                if cx_side is not None:
                    best_w = wgt
                    pass_cx = cx_side

        # Urgencia frontal por pared tiene prioridad sobre el sesgo de color
        # WRO — aquí no hay tiempo para respetar el lado de paso de una lata,
        # el chasis está apuntando de frente a la pared. Sin rampa: mientras
        # dure la condición, se fuerza el lado despejado con máxima urgencia.
        wall_cx = _wall_urgency_cx(row, wall_side) if wall_side != 0 else None

        if wall_cx is not None:
            cx = float(wall_cx)
            best_w = 1.0
            points.append((float(np.clip(cx, 0, w - 1)), y))
            weights.append(best_w)
            continue

        if free_cx is None and pass_cx is None:
            # Sin hueco válido en esta fila: no dejar un vacío en el path —
            # usar el pixel con mayor margen de seguridad (distanceTransform),
            # que ya calculamos arriba para safe_mask.
            row_dist = dist[y, :]
            if row_dist.max() > 0:
                cx_fallback = float(np.argmax(row_dist))
                points.append((float(np.clip(cx_fallback, 0, w - 1)), y))
                weights.append(best_w)
                continue

            # Fila SIN NADA de piso detectado (viendo la pared casi de frente
            # / ángulo muy oblicuo — dist.max()==0 significa que ni siquiera
            # hay un pixel libre en toda la fila, no hay CÓMO saber dónde
            # está el hueco a esta distancia específica).
            #
            # No basta con comprometerse "a ciegas" al lado que ya se sabía
            # abierto: la pared sigue una recta, y si ya se detectó que un
            # borde se viene cerrando fila a fila, esa MISMA tendencia sigue
            # más adelante -- se ajusta su pendiente con las dos últimas
            # lecturas reales (edge_hist) y se proyecta hacia esta fila,
            # aplicando el margen de seguridad contra la posición PREDICHA de
            # la pared (no solo la visible). Esto es lo que permite anticipar
            # que hay que cerrar el giro más de lo que el hueco visible
            # todavía sugiere.
            if len(edge_hist) >= 2:
                (y1, l1, r1), (y2, l2, r2) = edge_hist[-2], edge_hist[-1]
                dy = y2 - y1
                dl = (l2 - l1) / dy if dy != 0 else 0.0
                dr = (r2 - r1) / dy if dy != 0 else 0.0
                if abs(dr) >= abs(dl):
                    # Borde DERECHO cerrándose -> pared a la derecha.
                    predicted_r = r2 + dr * (y - y2)
                    cx_pred = predicted_r - C.WALL_MARGIN_PX
                else:
                    # Borde IZQUIERDO cerrándose -> pared a la izquierda.
                    predicted_l = l2 + dl * (y - y2)
                    cx_pred = predicted_l + C.WALL_MARGIN_PX
                points.append((float(np.clip(cx_pred, 0, w - 1)), y))
                weights.append(1.0)
                continue

            # Sin historial suficiente para ajustar una tendencia (p.ej. la
            # primera fila ya sale sin piso): comprometerse al lado que ya se
            # sabía abierto, con máxima urgencia, como red de seguridad.
            if points:
                last_x, _last_y = points[-1]
                side = -1 if last_x < C.ROBOT_BEV_X else 1
                cx_extrap = 0.0 if side < 0 else float(w - 1)
                points.append((cx_extrap, y))
                weights.append(1.0)
            continue

        if free_cx is None:
            assert pass_cx is not None
            cx = float(pass_cx)
        elif pass_cx is None or best_w <= 0.0:
            cx = float(free_cx)
        else:
            cx = (1.0 - best_w) * float(free_cx) + best_w * float(pass_cx)

        points.append((float(np.clip(cx, 0, w - 1)), y))
        weights.append(best_w)

    if points:
        points = _limit_lateral_step(points, weights)
        points = _smooth_x(points, weights)

    return [(int(round(np.clip(x, 0, w - 1))), int(y)) for x, y in points]


# ── Visualización BEV ─────────────────────────────────────────────────────────

def draw_bev_debug(
    bev_bgr: np.ndarray,
    path_points: list[tuple[int, int]],
    lookahead_pt: tuple[float, float] | None,
    bev_obstacles: list[tuple[float, float, str]],
    steer_deg: float = 0.0,
    pp_active: bool = False,
    line_info: dict | None = None,
    bev_obstacles_beyond: list[tuple[float, float, str]] | None = None,
) -> np.ndarray:
    """
    Dibuja sobre la imagen BEV:
      - Obstáculos con su radio de inflado
      - Centerline (puntos + línea)
      - Punto look-ahead (círculo amarillo)
      - Posición del robot con flecha de heading
      - Círculo del radio look-ahead
      - Texto de estado
      - Líneas de esquina (line_info, ver corner_lines.py) — si se pudo
        ajustar una recta con pendiente (OrangeLineTracker.classify()), se
        dibuja INCLINADA tal cual; si no (línea muy ocluida/corta para
        ajustar), cae de vuelta a una barra horizontal plana en near_y.
    """
    out = bev_bgr.copy()

    # Líneas de esquina
    if line_info:
        h, w = out.shape[:2]
        y_txt = 58
        for color, col_bgr in (("Orange", (0, 140, 255)),):
            info = line_info.get(color, {"seen": False, "near_y": None, "line": None})
            if info["seen"]:
                ny = int(info["near_y"])
                line = info.get("line")
                if line is not None and abs(line[0]) > 1e-3:
                    vx, vy, x0, y0 = line
                    slope = vy / vx
                    y_left  = int(round(y0 + (0     - x0) * slope))
                    y_right = int(round(y0 + (w - 1 - x0) * slope))
                    cv2.line(out, (0, y_left), (w - 1, y_right), col_bgr, 2)
                else:
                    cv2.line(out, (0, ny), (w, ny), col_bgr, 2)   # fallback plano
                close = (C.ROBOT_BEV_Y - ny) <= C.LINE_PROXIMITY_PX
                txt = f"{color}: y={ny}" + ("  CERCA" if close else "")
            else:
                txt = f"{color}: no visto"
            cv2.putText(out, txt, (6, y_txt), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col_bgr, 1)
            y_txt += 18

    # Obstáculos más allá de la naranja — atenuados, NO entran a detect_centerline()
    if bev_obstacles_beyond:
        for ox, oy, color in bev_obstacles_beyond:
            col_bgr = (0, 0, 90) if color == "Red" else (0, 90, 0)
            cv2.circle(out, (int(ox), int(oy)), C.OBS_PHYSICAL_R_PX, col_bgr, 1)
            cv2.circle(out, (int(ox), int(oy)), 3, col_bgr, -1)

    # Obstáculos (mi recta — los que sí afectan centerline/PP)
    for ox, oy, color in bev_obstacles:
        col_bgr = (0, 0, 200) if color == "Red" else (0, 200, 0)
        cv2.circle(out, (int(ox), int(oy)), C.OBS_INFLATE_R,   col_bgr, 1)   # zona bloqueada
        cv2.circle(out, (int(ox), int(oy)), C.OBS_PHYSICAL_R_PX, col_bgr, 2)  # tamaño real de lata
        cv2.circle(out, (int(ox), int(oy)), 4, col_bgr, -1)

    # Centerline
    if len(path_points) > 1:
        pts_arr = np.array(path_points, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [pts_arr], False, (0, 220, 220), 2)
    for pt in path_points:
        cv2.circle(out, pt, 3, (0, 220, 220), -1)

    # Punto look-ahead — solo cuando PP está activo (evita mostrar punto obsoleto en modo fallback)
    if lookahead_pt is not None and pp_active:
        lx, ly = int(lookahead_pt[0]), int(lookahead_pt[1])
        cv2.circle(out, (lx, ly), 8, (0, 220, 255), -1)
        cv2.circle(out, (lx, ly), 8, (0, 0, 0), 2)

    # Robot
    rx, ry = C.ROBOT_BEV_X, C.ROBOT_BEV_Y
    cv2.circle(out, (rx, ry), 9, (255, 80, 0), -1)
    cv2.arrowedLine(out, (rx, ry), (rx, ry - 25), (255, 255, 255), 2, tipLength=0.35)

    # Radio look-ahead
    cv2.circle(out, (rx, ry), int(C.LOOKAHEAD_PX), (60, 60, 60), 1)

    # Estado
    mode_txt = f"PP  steer={steer_deg:+.1f}deg" if pp_active else "FALLBACK PID"
    col_txt  = (0, 220, 0) if pp_active else (0, 100, 255)
    cv2.putText(out, mode_txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.50, col_txt, 2)
    cv2.putText(out, f"path_pts={len(path_points)}", (6, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    return out
