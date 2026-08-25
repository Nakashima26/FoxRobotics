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

def _widest_free_segment(row_mask: np.ndarray, min_width: int) -> int | None:
    """
    Retorna el centro-x del segmento libre más ancho en una fila de máscara.
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
    return (best_l + best_r) // 2


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

    # ── 3. Muestreo fila a fila (rampa al lado de paso, no switch binario) ────
        # ── 3. Muestreo fila a fila ────────────────────────────────────────────
    points: list[tuple[float, int]] = []
    weights: list[float] = []
    for y in range(h - 10, C.CENTERLINE_TOP_Y, -C.CENTERLINE_ROW_STEP):
        row = free_mask[y, :]

        free_cx = _widest_free_segment(safe_mask[y, :], C.CENTERLINE_MIN_WIDTH)
        if free_cx is None:
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
) -> np.ndarray:
    """
    Dibuja sobre la imagen BEV:
      - Obstáculos con su radio de inflado
      - Centerline (puntos + línea)
      - Punto look-ahead (círculo amarillo)
      - Posición del robot con flecha de heading
      - Círculo del radio look-ahead
      - Texto de estado
      - Líneas de esquina (line_info, ver corner_lines.detect_lines()) — una
        barra horizontal completa por color en su Y detectada, tal como se
        ve en corner_lines.py/config.LINE_*.
    """
    out = bev_bgr.copy()

    # Líneas de esquina
    if line_info:
        h, w = out.shape[:2]
        y_txt = 58
        for color, col_bgr in (("Orange", (0, 140, 255)),):
            info = line_info.get(color, {"seen": False, "near_y": None})
            if info["seen"]:
                ny = int(info["near_y"])
                cv2.line(out, (0, ny), (w, ny), col_bgr, 2)          # dónde está la línea
                close = (C.ROBOT_BEV_Y - ny) <= C.LINE_PROXIMITY_PX
                txt = f"{color}: y={ny}" + ("  CERCA" if close else "")
            else:
                txt = f"{color}: no visto"
            cv2.putText(out, txt, (6, y_txt), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col_bgr, 1)
            y_txt += 18

    # Obstáculos
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
