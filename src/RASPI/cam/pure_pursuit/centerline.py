"""
Detección de centerline en imagen BEV y mapeo de obstáculos a coordenadas BEV.

Algoritmo:
  1. Máscara de piso (HSV FLOOR_LOWER / FLOOR_UPPER) en la imagen BEV.
  2. Elimina zonas de obstáculos (inflado simétrico + sesgo asimétrico WRO).
  3. Muestrea filas de abajo hacia arriba y calcula el centroide X de cada
     franja libre → lista de puntos (x, y) = centerline.

Sesgo de color WRO:
  Rojo  → el robot pasa por la DERECHA del bloque
           → se bloquea más a la IZQUIERDA del bloque en BEV
  Verde → el robot pasa por la IZQUIERDA del bloque
           → se bloquea más a la DERECHA del bloque en BEV
"""

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


# ── Detección de centerline ───────────────────────────────────────────────────

def detect_centerline(
    bev_bgr: np.ndarray,
    bev_obstacles: list[tuple[float, float, str]],
) -> list[tuple[int, int]]:
    """
    Detecta la línea central del corredor en la imagen BEV.

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

    # ── 3. Muestreo fila a fila ───────────────────────────────────────────────
    points: list[tuple[int, int]] = []
    for y in range(h - 10, C.CENTERLINE_TOP_Y, -C.CENTERLINE_ROW_STEP):
        # Si hay obstáculo activo en esta fila, forzar punto en el lado correcto
        # sin consultar la máscara de piso.
        forced_cx: int | None = None
        for ox, oy, color in bev_obstacles:
            if abs(oy - y) <= C.OBS_INFLATE_R + C.OBS_BIAS_SHIFT:
                iox = int(ox)
                pref_min = max(1, C.CENTERLINE_MIN_WIDTH // 2)
                if color == "Red":
                    # Buscar centro del espacio libre a la derecha del obstáculo
                    cx_rel = _widest_free_segment(free_mask[y, iox:], pref_min)
                    # Fallback: si no hay segmento libre suficientemente ancho,
                    # usar el radio de seguridad INFLADO (no el físico) para
                    # nunca forzar un punto pegado al borde real de la lata.
                    forced_cx = (iox + cx_rel) if cx_rel is not None else iox + C.OBS_INFLATE_R
                elif color == "Green":
                    # Buscar centro del espacio libre a la izquierda del obstáculo
                    cx_abs = _widest_free_segment(free_mask[y, :iox], pref_min)
                    forced_cx = cx_abs if cx_abs is not None else iox - C.OBS_INFLATE_R
                break

        if forced_cx is not None:
            points.append((np.clip(forced_cx, 0, w - 1), y))
        else:
            cx = _widest_free_segment(free_mask[y, :], C.CENTERLINE_MIN_WIDTH)
            if cx is not None:
                points.append((cx, y))

    return points


# ── Visualización BEV ─────────────────────────────────────────────────────────

def draw_bev_debug(
    bev_bgr: np.ndarray,
    path_points: list[tuple[int, int]],
    lookahead_pt: tuple[float, float] | None,
    bev_obstacles: list[tuple[float, float, str]],
    steer_deg: float = 0.0,
    pp_active: bool = False,
    lookahead_px: float | None = None,
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

    # Radio look-ahead (dinámico: cambia según qué tan cerca esté el obstáculo)
    lk = lookahead_px if lookahead_px is not None else C.LOOKAHEAD_FAR_PX
    cv2.circle(out, (rx, ry), int(lk), (60, 60, 60), 1)

    # Estado
    mode_txt = f"PP  steer={steer_deg:+.1f}deg" if pp_active else "FALLBACK PID"
    col_txt  = (0, 220, 0) if pp_active else (0, 100, 255)
    cv2.putText(out, mode_txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.50, col_txt, 2)
    cv2.putText(out, f"path_pts={len(path_points)}", (6, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    return out
