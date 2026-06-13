"""
WRO Future Engineers — Simulacion v2
Arquitectura: BEV Occupancy Grid -> Trajectory Rollout -> Pure Pursuit
"""
import pygame
import math
import numpy as np

# ═══════════════════════════════ Display ══════════════════════════════════════
WIDTH, HEIGHT = 1200, 800
FPS           = 60
MAP_X0        = 800       # panel derecho empieza aqui
PANEL_W       = 400

WHITE     = (255, 255, 255)
BLACK     = (0,   0,   0  )
RED_C     = (255, 50,  50 )
GREEN_C   = (0,   200, 0  )
GRAY      = (200, 200, 200)
DARK_GRAY = (50,  50,  50 )
BLUE      = (0,   80,  255)
CYAN      = (0,   220, 220)
YELLOW    = (255, 220, 0  )

# ═══════════════════════════════ Track ════════════════════════════════════════
TRACK_CX, TRACK_CY = 400, 400
TRACK_OW, TRACK_OH = 720, 720
TRACK_IW, TRACK_IH = 240, 320

# ═══════════════════════════════ Robot ════════════════════════════════════════
WHEELBASE    = 40.0
VELOCIDAD    = 2.5
MAX_STEERING = 35.0
SLEW_RATE    = 2.0

INIT_X   = 200.0
INIT_Y   = 400.0
INIT_HDG = 90.0

# ═══════════════════════════════ Occupancy Grid ═══════════════════════════════
CELL_SZ     = 8         # pixeles por celda
GCOLS       = MAP_X0 // CELL_SZ    # 100 columnas (cubre x: 0..799)
GROWS       = HEIGHT  // CELL_SZ   # 100 filas    (cubre y: 0..799)
OBS_INFLATE = 3         # inflar obstaculos N celdas (margen de seguridad)

# ═══════════════════════════════ Trajectory Rollout ═══════════════════════════
N_TRAJ    = 21          # angulos de steering candidatos a evaluar
TRAJ_STEP = 100         # pasos de simulacion por trayectoria
TRAJ_DT   = 2.0         # tamano de paso (pixeles) — 100×2 = 200px look-ahead total

# ═══════════════════════════════ Pure Pursuit ══════════════════════════════════
LOOKAHEAD = 80.0         # look-ahead (px) — debe ser < TRAJ_STEP*TRAJ_DT

# ═══════════════════════════════ Color Bias (WRO rules) ═══════════════════════
COLOR_BIAS_DIST   = 300.0   # distancia maxima a la que un obstaculo influye
COLOR_BIAS_WEIGHT = 60.0    # peso maximo del sesgo de color vs heading (max ~60)

# ═══════════════════════════════ Corner Detection ═════════════════════════════
RAY_DIST            = 1000
FRONT_ANGLE_L       =   80
FRONT_ANGLE_R       =  -80
CORNER_WALL_THRESH  =  320
CORNER_ALIGN_THRESH =   25
OBS_CLEAR_FRAMES    =    8  # frames de debounce tras un giro

# ═══════════════════════════════ Heading Lock (memoria post-giro) ══════════════
HEADING_LOCK_FRAMES = 45   # frames que el heading queda "bloqueado" tras un giro
                            # durante ese tiempo el heading pesa 4x vs color bias

DEBUG_FONT    = "consolas"
DEBUG_FONT_SZ = 15
PRINT_EVERY   = 12


# ═══════════════════════════════ Helpers ══════════════════════════════════════
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def rect_from_center(cx, cy, w, h):
    return pygame.Rect(int(cx - w / 2), int(cy - h / 2), int(w), int(h))

def lerp_color(c1, c2, t):
    t = clamp(t, 0.0, 1.0)
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))


# ═══════════════════════════════ Occupancy Grid ═══════════════════════════════
def build_wall_grid(rect_ext, rect_int):
    """Construye la capa estatica de paredes. Se llama una sola vez al inicio."""
    grid = np.zeros((GROWS, GCOLS), dtype=np.uint8)
    for r in range(GROWS):
        for c in range(GCOLS):
            px = c * CELL_SZ + CELL_SZ // 2
            py = r * CELL_SZ + CELL_SZ // 2
            if not rect_ext.collidepoint(px, py) or rect_int.collidepoint(px, py):
                grid[r, c] = 1  # pared
    return grid

def update_obstacle_grid(wall_grid, obstacles):
    """Copia el wall_grid e infla los obstaculos de color encima."""
    grid = wall_grid.copy()
    for ox, oy, _ in obstacles:
        oc  = int(ox / CELL_SZ)
        or_ = int(oy / CELL_SZ)
        for dr in range(-OBS_INFLATE, OBS_INFLATE + 1):
            for dc in range(-OBS_INFLATE, OBS_INFLATE + 1):
                r, c = or_ + dr, oc + dc
                if 0 <= r < GROWS and 0 <= c < GCOLS:
                    grid[r, c] = 2  # obstaculo
    return grid

def cell_free(grid, x, y):
    """True si el pixel (x, y) cae en una celda libre."""
    c = int(x / CELL_SZ)
    r = int(y / CELL_SZ)
    if c < 0 or c >= GCOLS or r < 0 or r >= GROWS:
        return False
    return grid[r, c] == 0


# ═══════════════════════════════ Trajectory Rollout ═══════════════════════════
def rollout(sx, sy, s_hdg, steering_deg):
    """Simula modelo de bicicleta hacia adelante. Retorna lista de (x, y)."""
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
    """
    Sesgo direccional por reglas WRO:
      - Rojo  → pasar por la DERECHA del bloque (bloque a la izquierda del robot)
      - Verde → pasar por la IZQUIERDA del bloque (bloque a la derecha del robot)

    Usa producto cruzado en coords de pantalla (y hacia abajo):
      cross > 0  → obstaculo a la DERECHA del vector de avance
      cross < 0  → obstaculo a la IZQUIERDA
    """
    if not pts or not obstacles:
        return 0.0

    # Direccion del arco en coords de pantalla
    fwd_x = pts[-1][0] - pts[0][0]
    fwd_y = pts[-1][1] - pts[0][1]
    norm  = math.hypot(fwd_x, fwd_y)
    if norm < 1:
        return 0.0
    fwd_x /= norm
    fwd_y /= norm

    ox0, oy0 = pts[0]
    score = 0.0

    for ox, oy, col in obstacles:
        to_x = ox - ox0
        to_y = oy - oy0
        dist = math.hypot(to_x, to_y)
        if dist > COLOR_BIAS_DIST:
            continue

        # Producto cruzado 2D (pantalla): fwd × to_obs
        cross = fwd_x * to_y - fwd_y * to_x
        weight = (1.0 - dist / COLOR_BIAS_DIST) * COLOR_BIAS_WEIGHT

        if col == RED_C:
            # Queremos bloque a la IZQUIERDA (cross < 0): recompensar cross negativo
            score -= cross * weight / max(dist, 1.0)
        elif col == GREEN_C:
            # Queremos bloque a la DERECHA (cross > 0): recompensar cross positivo
            score += cross * weight / max(dist, 1.0)

    return clamp(score, -COLOR_BIAS_WEIGHT, COLOR_BIAS_WEIGHT)


def score_traj(pts, grid, target_hdg, obstacles, heading_bonus=0.0):
    """
    Puntua una trayectoria. Mayor = mejor. Colision -> muy negativo.
    heading_bonus: puntos extra al heading alignment post-giro (0..40).
      Se SUMA al heading score sin tocar el color bias — el rojo/verde
      siempre compite en igualdad de condiciones.
    """
    for i, (x, y) in enumerate(pts):
        if not cell_free(grid, x, y):
            return -1000.0 + i   # preferir colisiones mas tarde

    score = 0.0

    # Alineacion de heading final con target + bonus de memoria post-giro
    if len(pts) >= 2:
        dx = pts[-1][0] - pts[-2][0]
        dy = pts[-1][1] - pts[-2][1]
        final_hdg = math.degrees(math.atan2(-dy, dx))
        err = abs((final_hdg - target_hdg + 180) % 360 - 180)
        score += max(0.0, 60.0 + heading_bonus - err * 1.5)

    # Sesgo de color — independiente del heading lock, siempre activo
    score += color_bias(pts, obstacles)

    return score


def trajectory_rollout(rx, ry, hdg, grid, target_hdg, obstacles, heading_bonus=0.0):
    """
    Evalua N_TRAJ angulos candidatos.
    Retorna (mejor_path, lista ordenada de (score, path, steering)).
    """
    results = []
    for i in range(N_TRAJ):
        steer = -MAX_STEERING + i * (2 * MAX_STEERING / (N_TRAJ - 1))
        pts   = rollout(rx, ry, hdg, steer)
        sc    = score_traj(pts, grid, target_hdg, obstacles, heading_bonus)
        results.append((sc, pts, steer))
    results.sort(key=lambda x: x[0], reverse=True)
    return results[0][1], results


# ═══════════════════════════════ Pure Pursuit ══════════════════════════════════
def pure_pursuit(rx, ry, hdg, path):
    """
    Controlador geometrico Pure Pursuit.
    Busca el primer punto en `path` a distancia >= LOOKAHEAD y calcula:
        steer = atan( 2 * L * sin(alpha) / lookahead_dist )
    Retorna (steering_deg, lookahead_point).
    """
    if not path:
        return 0.0, (rx, ry)

    target = path[-1]
    for pt in path:
        if math.hypot(pt[0] - rx, pt[1] - ry) >= LOOKAHEAD:
            target = pt
            break

    dx  = target[0] - rx
    dy  = target[1] - ry
    ld  = max(1.0, math.hypot(dx, dy))

    world_angle = math.degrees(math.atan2(-dy, dx))
    alpha       = (world_angle - hdg + 180) % 360 - 180  # signed [-180, 180]

    steer = math.degrees(math.atan2(2.0 * WHEELBASE * math.sin(math.radians(alpha)), ld))
    return clamp(steer, -MAX_STEERING, MAX_STEERING), target


# ═══════════════════════════════ Corner Detection ═════════════════════════════
def ray_to_wall(rx, ry, ang_deg, rect_ext, rect_int):
    rad = math.radians(ang_deg)
    ex  = rx + math.cos(rad) * RAY_DIST
    ey  = ry - math.sin(rad) * RAY_DIST
    d   = float(RAY_DIST)

    clip = rect_ext.clipline(rx, ry, ex, ey)
    if clip:
        d1 = math.hypot(clip[0][0] - rx, clip[0][1] - ry)
        d2 = math.hypot(clip[1][0] - rx, clip[1][1] - ry)
        d  = min(d, max(d1, d2))

    clip = rect_int.clipline(rx, ry, ex, ey)
    if clip:
        d1 = math.hypot(clip[0][0] - rx, clip[0][1] - ry)
        d2 = math.hypot(clip[1][0] - rx, clip[1][1] - ry)
        d  = min(d, min(d1, d2))

    return max(1.0, d)


# ═══════════════════════════════ Rendering ════════════════════════════════════
def draw_debug(surface, lines, x, y, color=WHITE, gap=20):
    for line in lines:
        surf = font_debug.render(line, True, color)
        surface.blit(surf, (x, y))
        y += gap


# ═══════════════════════════════ Init ════════════════════════════════════════
pygame.init()
screen     = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("WRO Sim v2 — Occupancy Grid + Trajectory Rollout + Pure Pursuit")
clock      = pygame.time.Clock()
font_debug = pygame.font.SysFont(DEBUG_FONT, DEBUG_FONT_SZ)

RECT_EXT  = rect_from_center(TRACK_CX, TRACK_CY, TRACK_OW, TRACK_OH)
RECT_INT  = rect_from_center(TRACK_CX, TRACK_CY, TRACK_IW, TRACK_IH)
wall_grid = build_wall_grid(RECT_EXT, RECT_INT)  # se computa una sola vez

obstacles = []

def reset_state():
    global rx, ry, heading, target_heading
    global prev_steer, turn_counter, race_finished, obs_clear_frames, frame_n
    global heading_lock_remaining
    rx, ry                = INIT_X, INIT_Y
    heading               = INIT_HDG
    target_heading        = INIT_HDG
    prev_steer            = 0.0
    turn_counter          = 0
    heading_lock_remaining = 0
    race_finished    = False
    obs_clear_frames = 0
    frame_n          = 0

reset_state()

# estado de render inicializado para evitar referencias antes del primer frame
best_path              = []
all_trajs              = []
lookahead_pt           = (INIT_X, INIT_Y)
steer                  = 0.0
heading_lock_remaining = 0
heading_bonus          = 0.0

# ═══════════════════════════════ Main Loop ════════════════════════════════════
running = True
while running:
    frame_n += 1

    # ── Eventos ───────────────────────────────────────────────────────────────
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()
            if mx < MAP_X0:
                if event.button == 1:
                    obstacles.append([mx, my, RED_C])
                elif event.button == 3:
                    obstacles.append([mx, my, GREEN_C])
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                reset_state()
            elif event.key == pygame.K_x and obstacles:
                mx, my = pygame.mouse.get_pos()
                obstacles.sort(key=lambda o: math.hypot(o[0] - mx, o[1] - my))
                obstacles.pop(0)

    # ── 1. Occupancy Grid ─────────────────────────────────────────────────────
    grid = update_obstacle_grid(wall_grid, obstacles)

    # ── Eje delantero ─────────────────────────────────────────────────────────
    rad_hdg = math.radians(heading)
    front_x = rx + WHEELBASE * math.cos(rad_hdg)
    front_y = ry - WHEELBASE * math.sin(rad_hdg)

    # ── Corner Detection ──────────────────────────────────────────────────────
    align_err = abs((heading - target_heading + 180) % 360 - 180)
    turned = False

    if not race_finished and obs_clear_frames >= OBS_CLEAR_FRAMES and align_err < CORNER_ALIGN_THRESH:
        dl = ray_to_wall(front_x, front_y, heading + FRONT_ANGLE_L, RECT_EXT, RECT_INT)
        dr = ray_to_wall(front_x, front_y, heading + FRONT_ANGLE_R, RECT_EXT, RECT_INT)

        if dl > CORNER_WALL_THRESH and dr < CORNER_WALL_THRESH:
            target_heading = (target_heading + 90 + 180) % 360 - 180
            turn_counter  += 1
            turned = True
        elif dr > CORNER_WALL_THRESH and dl < CORNER_WALL_THRESH:
            target_heading = (target_heading - 90 + 180) % 360 - 180
            turn_counter  += 1
            turned = True

        if turn_counter >= 12:
            race_finished = True

    if turned:
        obs_clear_frames = 0
        heading_lock_remaining = HEADING_LOCK_FRAMES   # guardar angulo en memoria
    else:
        obs_clear_frames += 1
        heading_lock_remaining = max(0, heading_lock_remaining - 1)

    # heading_bonus: +40 pts recien girado, baja linealmente hasta 0
    # El color bias NO se toca — rojo/verde siempre funcionan
    heading_bonus = 40.0 * (heading_lock_remaining / HEADING_LOCK_FRAMES)

    # ── 2. Trajectory Rollout ─────────────────────────────────────────────────
    best_path, all_trajs = trajectory_rollout(
        rx, ry, heading, grid, target_heading, obstacles, heading_bonus
    )

    # ── 3. Pure Pursuit ───────────────────────────────────────────────────────
    if not race_finished:
        raw_steer, lookahead_pt = pure_pursuit(rx, ry, heading, best_path)
    else:
        raw_steer = 0.0

    # Slew rate limiter (limitador de velocidad del servo)
    delta     = raw_steer - prev_steer
    raw_steer = prev_steer + clamp(delta, -SLEW_RATE, SLEW_RATE)
    steer     = clamp(raw_steer, -MAX_STEERING, MAX_STEERING)
    prev_steer = steer

    # ── Cinematica de bicicleta (Ackermann) ───────────────────────────────────
    if not race_finished:
        rad_s    = math.radians(steer)
        turn_int = min(1.0, abs(steer) / MAX_STEERING)
        speed    = max(0.6, VELOCIDAD * (1.0 - 0.45 * turn_int))

        d_hdg   = math.degrees((speed / WHEELBASE) * math.tan(rad_s))
        heading = (heading + d_hdg + 180) % 360 - 180
        rad_hdg = math.radians(heading)
        rx     += speed * math.cos(rad_hdg)
        ry     -= speed * math.sin(rad_hdg)

    # ── Render: Mapa ──────────────────────────────────────────────────────────
    screen.fill(GRAY)
    pygame.draw.rect(screen, WHITE, RECT_EXT)
    pygame.draw.rect(screen, BLACK, RECT_EXT, 8)
    pygame.draw.rect(screen, GRAY,  RECT_INT)
    pygame.draw.rect(screen, BLACK, RECT_INT, 8)

    for ox, oy, col in obstacles:
        pygame.draw.rect(screen, col, (ox - 10, oy - 10, 20, 20))

    # Trayectorias candidatas coloreadas por puntaje (rojo=mala, verde=buena)
    scores  = [s for s, _, _ in all_trajs]
    s_min, s_max = min(scores), max(scores)
    s_range = max(1.0, s_max - s_min)
    for sc, pts, _ in all_trajs:
        if len(pts) > 1:
            t   = (sc - s_min) / s_range
            col = lerp_color((140, 0, 0), (0, 140, 0), t)
            pygame.draw.lines(screen, col, False, [(int(p[0]), int(p[1])) for p in pts], 1)

    # Mejor trayectoria (cyan, mas gruesa)
    if len(best_path) > 1:
        pygame.draw.lines(screen, CYAN, False,
                          [(int(p[0]), int(p[1])) for p in best_path], 3)

    # Circulo de vision del rollout (radio = TRAJ_STEP * TRAJ_DT)
    vision_r = int(TRAJ_STEP * TRAJ_DT)
    vis_surf = pygame.Surface((vision_r * 2, vision_r * 2), pygame.SRCALPHA)
    pygame.draw.circle(vis_surf, (255, 255, 255, 18), (vision_r, vision_r), vision_r)
    pygame.draw.circle(vis_surf, (255, 255, 255, 60), (vision_r, vision_r), vision_r, 1)
    screen.blit(vis_surf, (int(rx) - vision_r, int(ry) - vision_r))

    # Punto look-ahead del Pure Pursuit
    pygame.draw.circle(screen, YELLOW, (int(lookahead_pt[0]), int(lookahead_pt[1])), 7)
    pygame.draw.circle(screen, BLACK,  (int(lookahead_pt[0]), int(lookahead_pt[1])), 7, 2)

    # Chasis del robot
    rad_hdg  = math.radians(heading)
    front_x2 = rx + WHEELBASE * math.cos(rad_hdg)
    front_y2 = ry - WHEELBASE * math.sin(rad_hdg)
    pygame.draw.line(screen, BLACK, (int(rx), int(ry)), (int(front_x2), int(front_y2)), 4)
    pygame.draw.circle(screen, BLUE, (int(rx), int(ry)), 7)
    rad_w = math.radians(heading + steer)
    wx1 = front_x2 + 9 * math.cos(rad_w);  wy1 = front_y2 - 9 * math.sin(rad_w)
    wx2 = front_x2 - 9 * math.cos(rad_w);  wy2 = front_y2 + 9 * math.sin(rad_w)
    pygame.draw.line(screen, RED_C, (int(wx1), int(wy1)), (int(wx2), int(wy2)), 6)

    # ── Render: Panel derecho — Occupancy Grid (Bird's Eye View) ─────────────
    panel = pygame.Surface((PANEL_W, HEIGHT))
    panel.fill(DARK_GRAY)

    # Mini-mapa del occupancy grid escalado (800px → 400px, escala 0.5)
    MINI_W = PANEL_W
    MINI_H = MINI_W
    MINI_S = MINI_W / MAP_X0           # 0.5
    mini_cell = max(1, int(CELL_SZ * MINI_S))

    # Solo dibuja celdas ocupadas (mucho mas rapido que iterar todas)
    occ = np.argwhere(grid > 0)
    for r, c in occ:
        px  = int(c * CELL_SZ * MINI_S)
        py  = int(r * CELL_SZ * MINI_S)
        col = (70, 70, 70) if grid[r, c] == 1 else (200, 70, 70)
        pygame.draw.rect(panel, col, (px, py, mini_cell, mini_cell))

    # Mejor trayectoria en mini-mapa
    if len(best_path) > 1:
        mpts = [(int(p[0] * MINI_S), int(p[1] * MINI_S)) for p in best_path]
        pygame.draw.lines(panel, YELLOW, False, mpts, 2)

    # Robot en mini-mapa con flecha de heading
    mrx, mry = int(rx * MINI_S), int(ry * MINI_S)
    pygame.draw.circle(panel, CYAN, (mrx, mry), 5)
    hax = int((rx + 15 * math.cos(rad_hdg)) * MINI_S)
    hay = int((ry - 15 * math.sin(rad_hdg)) * MINI_S)
    pygame.draw.line(panel, WHITE, (mrx, mry), (hax, hay), 2)

    # Punto lookahead en mini-mapa
    pygame.draw.circle(panel, YELLOW,
                       (int(lookahead_pt[0] * MINI_S), int(lookahead_pt[1] * MINI_S)), 4)

    pygame.draw.line(panel, (100, 100, 100), (0, MINI_H), (PANEL_W, MINI_H), 1)

    # Debug text
    best_sc = all_trajs[0][0] if all_trajs else 0.0
    best_s  = all_trajs[0][2] if all_trajs else 0.0
    lap_str = "TERMINADO" if race_finished else f"{turn_counter // 4}/3"
    draw_debug(panel, [
        "BEV Occupancy Grid",
        "Trajectory Rollout + Pure Pursuit",
        "─────────────────────────────────",
        f"heading  : {heading:7.1f} deg",
        f"target   : {target_heading:7.1f} deg",
        f"align err: {align_err:7.1f} deg",
        f"steer    : {steer:7.1f} deg",
        f"best steer (rollout): {best_s:.1f}",
        f"traj score: {best_sc:.1f}",
        f"hdg lock : {heading_lock_remaining:3d} frames  +{heading_bonus:.0f}pts",
        f"turns    : {turn_counter}/12   laps: {lap_str}",
        f"obstacles: {len(obstacles)}",
        f"vision r : {int(TRAJ_STEP * TRAJ_DT)}px  lookahead: {int(LOOKAHEAD)}px",
        f"lookahead pt: ({lookahead_pt[0]:.0f}, {lookahead_pt[1]:.0f})",
        "",
        "Controles:",
        "  R       — reset",
        "  Click L — obstaculo rojo",
        "  Click R — obstaculo verde",
        "  X       — eliminar mas cercano",
    ], 8, MINI_H + 10, WHITE, 20)

    screen.blit(panel, (MAP_X0, 0))
    pygame.draw.line(screen, BLACK, (MAP_X0, 0), (MAP_X0, HEIGHT), 2)

    # Banner de carrera terminada
    if race_finished:
        font_big = pygame.font.SysFont(DEBUG_FONT, 44, bold=True)
        txt = font_big.render("TERMINADO - 3 VUELTAS", True, (0, 255, 0))
        screen.blit(txt, (MAP_X0 // 2 - txt.get_width() // 2, HEIGHT // 2 - 24))

    if frame_n % PRINT_EVERY == 0:
        print(
            f"[v2] hdg={heading:.1f} tgt={target_heading:.1f} steer={steer:.1f} "
            f"score={best_sc:.1f} turns={turn_counter} laps={turn_counter//4}",
            flush=True,
        )

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
