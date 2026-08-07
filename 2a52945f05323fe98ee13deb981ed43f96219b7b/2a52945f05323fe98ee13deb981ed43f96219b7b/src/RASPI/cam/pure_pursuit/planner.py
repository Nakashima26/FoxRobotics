import math
import cv2
import numpy as np

# =====================================================================
# AJUSTES VITALES (Modifica estos según tu cámara BEV)
# =====================================================================
# Si tu lata mide 5cm de diámetro, el radio es de 2.5cm.
# Tienes que medir cuántos pixeles en tu imagen BEV equivalen a 1 cm.
PIXELS_PER_CM   = 3.0  # <--- MIDE ESTO EN TU IMAGEN BEV
OBS_RADIUS_CM   = 2.5  
OBS_INFLATE_PX  = int(OBS_RADIUS_CM * PIXELS_PER_CM)

# Ajustes del Rollout
N_TRAJ            = 21
TRAJ_STEP         = 40    # Qué tan lejos mira a futuro
TRAJ_DT           = 3.0   # Tamaño del paso en pixeles
MAX_STEERING      = 35.0
WHEELBASE         = 30.0  # Distancia entre llantas en pixeles BEV
COLOR_BIAS_DIST   = 150.0 # Qué tan lejos "ve" los colores
COLOR_BIAS_WEIGHT = 60.0

def rollout(sx, sy, s_hdg, steering_deg):
    """Simula el auto hacia adelante usando el modelo de bicicleta."""
    x, y, hdg = sx, sy, s_hdg
    pts = []
    rad_s = math.radians(steering_deg)
    for _ in range(TRAJ_STEP):
        d_hdg = math.degrees((TRAJ_DT / WHEELBASE) * math.tan(rad_s))
        hdg   = (hdg + d_hdg + 180) % 360 - 180
        x    += TRAJ_DT * math.cos(math.radians(hdg))
        y    -= TRAJ_DT * math.sin(math.radians(hdg))
        pts.append((x, y))
    return pts

def color_bias(pts, obstacles):
    """Aplica la regla WRO: Verde = pasar por izquierda, Rojo = pasar por derecha."""
    if not pts or not obstacles: return 0.0
    fwd_x = pts[-1][0] - pts[0][0]
    fwd_y = pts[-1][1] - pts[0][1]
    norm  = math.hypot(fwd_x, fwd_y)
    if norm < 1: return 0.0
    fwd_x, fwd_y = fwd_x / norm, fwd_y / norm
    
    ox0, oy0 = pts[0]
    score = 0.0

    for ox, oy, col in obstacles:
        to_x, to_y = ox - ox0, oy - oy0
        dist = math.hypot(to_x, to_y)
        if dist > COLOR_BIAS_DIST: continue

        cross = fwd_x * to_y - fwd_y * to_x
        weight = (1.0 - dist / COLOR_BIAS_DIST) * COLOR_BIAS_WEIGHT

        if col == "Red":
            score -= cross * weight / max(dist, 1.0)
        elif col == "Green":
            score += cross * weight / max(dist, 1.0)

    return max(min(score, COLOR_BIAS_WEIGHT), -COLOR_BIAS_WEIGHT)

def is_free(grid, x, y):
    """Verifica si el pixel x, y choca contra una pared (valor 255)."""
    h, w = grid.shape
    ix, iy = int(x), int(y)
    if ix < 0 or ix >= w or iy < 0 or iy >= h: return False
    return grid[iy, ix] == 0  # 0 es libre, 255 es pared/obstáculo

def score_traj(pts, grid, target_hdg, obstacles):
    """Puntúa la trayectoria. Mayor = mejor."""
    for i, (x, y) in enumerate(pts):
        # AÑADIDO: 'i > 3' crea un escudo en los primeros 3 pasos para salir de sombras
        if i > 3 and not is_free(grid, x, y):
            return -1000.0 + i  # Chocó: castigo severo

    score = 0.0
    if len(pts) >= 2:
        dx = pts[-1][0] - pts[-2][0]
        dy = pts[-1][1] - pts[-2][1]
        final_hdg = math.degrees(math.atan2(-dy, dx))
        err = abs((final_hdg - target_hdg + 180) % 360 - 180)
        score += max(0.0, 60.0 - err * 1.5)

    score += color_bias(pts, obstacles)
    return score

def plan_trajectory(grid, rx, ry, heading, obstacles):
    """Genera 21 rutas, las califica, y devuelve la mejor."""
    results = []
    for i in range(N_TRAJ):
        steer = -MAX_STEERING + i * (2 * MAX_STEERING / (N_TRAJ - 1))
        pts   = rollout(rx, ry, heading, steer)
        sc    = score_traj(pts, grid, heading, obstacles)
        results.append((sc, pts, steer))
    
    results.sort(key=lambda x: x[0], reverse=True)
    return results[0][1], results  # best_path, todas_las_rutas
