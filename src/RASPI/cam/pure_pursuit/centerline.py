"""
Detección de centerline en imagen BEV y mapeo de obstáculos a coordenadas BEV.

NOTA: la esquiva "fina" de obstáculos ya NO se hace aquí — la hace
avoidance.py por cinemática inversa (tangente). Esta función solo enmascara
los obstáculos de forma SIMÉTRICA para que, en modo centerline puro (sin
esquiva activa, o durante la transición de blend), el camino no cruce
literalmente sobre un obstáculo recordado.
"""

import cv2
import numpy as np

from .bev import BEVTransformer
from . import config as C


def map_obstacle_to_bev(bev, cam_x, cam_y, cam_w, cam_h):
    """Proyecta el punto de contacto con el suelo del obstáculo a BEV."""
    foot_x = cam_x + cam_w * 0.5
    foot_y = cam_y + cam_h
    result = bev.cam_to_bev(foot_x, foot_y)
    if result is None:
        return None
    bx, by = result
    if bev.bev_in_bounds(bx, by):
        return bx, by
    return None


def _widest_free_segment(row_mask, min_width):
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


def detect_centerline(bev_bgr, bev_obstacles):
    """
    Detecta la línea central del corredor en la imagen BEV.
    Obstáculos se enmascaran de forma simétrica (sin sesgo) — la esquiva
    precisa la maneja avoidance.py cuando el obstáculo está en rango de acción.
    """
    h, w = bev_bgr.shape[:2]

    hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
    floor_mask = cv2.inRange(hsv, C.FLOOR_LOWER, C.FLOOR_UPPER)

    k3 = np.ones((3, 3), np.uint8)
    k5 = np.ones((5, 5), np.uint8)
    floor_mask = cv2.morphologyEx(floor_mask, cv2.MORPH_OPEN,  k3)
    floor_mask = cv2.morphologyEx(floor_mask, cv2.MORPH_CLOSE, k5)

    free_mask = floor_mask.copy()
    for ox, oy, color in bev_obstacles:
        ix, iy = int(round(ox)), int(round(oy))
        cv2.circle(free_mask, (ix, iy), C.OBS_INFLATE_R, 0, -1)

    points = []
    for y in range(h - 10, C.CENTERLINE_TOP_Y, -C.CENTERLINE_ROW_STEP):
        cx = _widest_free_segment(free_mask[y, :], C.CENTERLINE_MIN_WIDTH)
        if cx is not None:
            points.append((cx, y))

    return points


def draw_bev_debug(bev_bgr, path_points, lookahead_pt, bev_obstacles,
                    steer_deg=0.0, pp_active=False, avoid_active=False):
    out = bev_bgr.copy()

    for ox, oy, color in bev_obstacles:
        col_bgr = (0, 0, 200) if color == "Red" else (0, 200, 0)
        cv2.circle(out, (int(ox), int(oy)), C.AVOID_RADIUS_PX,     col_bgr, 1)
        cv2.circle(out, (int(ox), int(oy)), C.OBS_PHYSICAL_R_PX,   col_bgr, 2)
        cv2.circle(out, (int(ox), int(oy)), 4, col_bgr, -1)

    if len(path_points) > 1:
        pts_arr = np.array(path_points, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [pts_arr], False, (0, 220, 220), 2)
    for pt in path_points:
        cv2.circle(out, pt, 3, (0, 220, 220), -1)

    if lookahead_pt is not None and pp_active:
        lx, ly = int(lookahead_pt[0]), int(lookahead_pt[1])
        col = (0, 150, 255) if avoid_active else (0, 220, 255)
        cv2.circle(out, (lx, ly), 8, col, -1)
        cv2.circle(out, (lx, ly), 8, (0, 0, 0), 2)

    rx, ry = C.ROBOT_BEV_X, C.ROBOT_BEV_Y
    cv2.circle(out, (rx, ry), 9, (255, 80, 0), -1)
    cv2.arrowedLine(out, (rx, ry), (rx, ry - 25), (255, 255, 255), 2, tipLength=0.35)
    cv2.circle(out, (rx, ry), int(C.LOOKAHEAD_PX), (60, 60, 60), 1)
    cv2.circle(out, (rx, ry), int(C.AVOID_ACTION_RANGE_PX), (90, 40, 40), 1)

    if avoid_active:
        mode_txt = f"AVOID(tangente)  steer={steer_deg:+.1f}deg"
        col_txt = (0, 150, 255)
    elif pp_active:
        mode_txt = f"PP  steer={steer_deg:+.1f}deg"
        col_txt = (0, 220, 0)
    else:
        mode_txt = "FALLBACK PID"
        col_txt = (0, 100, 255)

    cv2.putText(out, mode_txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.50, col_txt, 2)
    cv2.putText(out, f"path_pts={len(path_points)}", (6, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    return out