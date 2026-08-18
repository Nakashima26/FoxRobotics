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


def _pass_side_cx(row_mask: np.ndarray, ox: float, color: str) -> int | None:
    """Centro del hueco libre del lado WRO correcto en esta fila."""
    iox = int(ox)
    iox = max(0, min(iox, row_mask.shape[0] - 1))
    pref_min = max(1, C.CENTERLINE_MIN_WIDTH // 2)
    if color == "Red":
        cx_rel = _widest_free_segment(row_mask[iox:], pref_min)
        return (iox + cx_rel) if cx_rel is not None else iox + C.OBS_PHYSICAL_R_PX
    if color == "Green":
        cx_abs = _widest_free_segment(row_mask[:iox], pref_min)
        return cx_abs if cx_abs is not None else iox - C.OBS_PHYSICAL_R_PX
    return None


def _smooth_x(points: list[tuple[float, int]]) -> list[tuple[float, int]]:
    """Media móvil corta en X con padding de borde (no tira los extremos a 0)."""
    n = len(points)
    win = int(C.CENTERLINE_SMOOTH_WIN)
    if n < 3 or win < 3:
        return points
    if win % 2 == 0:
        win += 1
    xs = np.array([p[0] for p in points], dtype=np.float64)
    kernel = np.ones(win, dtype=np.float64) / win
    pad = win // 2
    padded = np.pad(xs, pad, mode="edge")
    xs_s = np.convolve(padded, kernel, mode="valid")
    return [(float(xs_s[i]), points[i][1]) for i in range(n)]


def _limit_lateral_step(points: list[tuple[float, int]]) -> list[tuple[float, int]]:
    """
    Acota |Δx| entre filas consecutivas a lo que el servo puede (tan(δ_max)·Δy).
    Recorre desde el robot (primer punto) hacia adelante.
    """
    if len(points) < 2:
        return points
    max_dx = math.tan(math.radians(C.MAX_STEER_DEG)) * float(C.CENTERLINE_ROW_STEP)
    out: list[tuple[float, int]] = [points[0]]
    prev_x = float(points[0][0])
    for i in range(1, len(points)):
        x, y = points[i]
        dx = x - prev_x
        if dx > max_dx:
            x = prev_x + max_dx
        elif dx < -max_dx:
            x = prev_x - max_dx
        out.append((x, y))
        prev_x = x
    return out


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
    floor_mask = cv2.inRange(hsv, C.FLOOR_LOWER, C.FLOOR_UPPER)

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

    # ── 3. Muestreo fila a fila (rampa al lado de paso, no switch binario) ────
    raw_points: list[tuple[float, int]] = []
    for y in range(h - 10, C.CENTERLINE_TOP_Y, -C.CENTERLINE_ROW_STEP):
        base_cx = _widest_free_segment(floor_mask[y, :], C.CENTERLINE_MIN_WIDTH)
        if base_cx is None:
            continue  # sin piso libre en esta fila, se omite

        best_w = 0.0
        best_cx = float(base_cx)
        for ox, oy, color in bev_obstacles:
            w = _ramp_weight(y, oy)
            if w <= 0.0:
                continue
            side_cx = _pass_side_cx(free_mask[y, :], ox, color)
            if side_cx is None:
                continue
            if w > best_w:
                best_w = w
                best_cx = base_cx + w * (side_cx - base_cx)

        raw_points.append((best_cx, y))

    points_f = _smooth_x(raw_points)
    points_f = _limit_lateral_step(points_f)
    return [(int(round(x)), y) for x, y in points_f]

# ── Visualización BEV ─────────────────────────────────────────────────────────

def draw_bev_debug(
    bev_bgr: np.ndarray,
    path_points: list[tuple[int, int]],
    lookahead_pt: tuple[float, float] | None,
    bev_obstacles: list[tuple[float, float, str]],
    steer_deg: float = 0.0,
    pp_active: bool = False,
) -> np.ndarray:
    """
    Dibuja sobre la imagen BEV:
      - Obstáculos con su radio de inflado
      - Centerline (puntos + línea)
      - Punto look-ahead (círculo amarillo)
      - Posición del robot con flecha de heading
      - Círculo del radio look-ahead
      - Texto de estado
    """
    out = bev_bgr.copy()

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
