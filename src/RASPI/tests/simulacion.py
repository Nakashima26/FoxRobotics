import pygame
import math

WIDTH, HEIGHT = 1200, 800
FPS = 60
WHITE, BLACK, RED, GREEN, GRAY, DARK_GRAY, BLUE = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0), (200, 200, 200), (50, 50, 50), (0, 0, 255)
DEBUG = True
DEBUG_PRINT_EVERY = 12

MAP_X0 = 800
PANEL_WIDTH = 400
PANEL_HEIGHT = 400
POV_HEIGHT = 200
DEBUG_PANEL_Y = 410
DEBUG_PANEL_ALPHA = 170
PADDING = 10

PIXELS_PER_VIEW = 640
CAMERA_FOV_DEG = 90
RAYS = 100
VIEW_DISTANCE = 300
RAY_DISTANCE = 1000
RAY_WALL_SCALE = 80

WALL_FRONT_ANGLE_LEFT = 80
WALL_FRONT_ANGLE_RIGHT = -80
WALL_SIDE_ANGLE_LEFT = 90
WALL_SIDE_ANGLE_RIGHT = -90
CORNER_ALIGN_THRESHOLD = 25
CORNER_WALL_THRESHOLD = 320
CENTER_ALIGN_THRESHOLD = 15
CENTER_WALL_THRESHOLD = 250

RED_PIXEL_THRESHOLD = 620
GREEN_PIXEL_THRESHOLD = 140
RED_PIXEL_DRAW_LINE = 140
GREEN_PIXEL_DRAW_LINE = 500
OBSTACLE_CLEAR_FRAMES = 10
OBSTACLE_HOLD_BAND = 24
OBSTACLE_MEMORY_FRAMES = 18
OBSTACLE_FADE_FACTOR = 0.85
OBSTACLE_MAX_CORRECTION = 35
VISION_MAX_CORRECTION = 45
CENTER_MAX_CORRECTION = 20

STEERING_LIMIT = 35.0
STEERING_SLEW_RATE = 1.25
WHEELBASE = 40.0
VELOCIDAD = 2.0
TURN_SPEED_FACTOR = 0.55
AVOIDANCE_SPEED_FACTOR = 0.85
OBSTACLE_PRIORITY_SPEED_FACTOR = 0.55

INITIAL_ROBOT_X = 200.0
INITIAL_ROBOT_Y = 400.0
INITIAL_HEADING = 90.0
INITIAL_TARGET_HEADING = 90.0

DEBUG_FONT_NAME = "consolas"
DEBUG_FONT_SIZE = 16

TRACK_CENTER_X = 400
TRACK_CENTER_Y = 400
TRACK_OUTER_WIDTH = 720
TRACK_OUTER_HEIGHT = 720
TRACK_INNER_WIDTH = 240
TRACK_INNER_HEIGHT = 320


def rect_from_center(center_x, center_y, width, height):
    return pygame.Rect(
        int(center_x - width / 2),
        int(center_y - height / 2),
        int(width),
        int(height),
    )


RECT_EXTERIOR = rect_from_center(
    TRACK_CENTER_X,
    TRACK_CENTER_Y,
    TRACK_OUTER_WIDTH,
    TRACK_OUTER_HEIGHT,
)
RECT_INTERIOR = rect_from_center(
    TRACK_CENTER_X,
    TRACK_CENTER_Y,
    TRACK_INNER_WIDTH,
    TRACK_INNER_HEIGHT,
)

class PID:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0

    def calcular(self, error):
        derivada = error - self.prev_error
        self.prev_error = error
        return (self.kp * error) + (self.kd * derivada)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Simulador WRO - Físicas Ackermann Reales (Carro RC)")
clock = pygame.time.Clock()


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def reset_car_state():
    global robot_x, robot_y, robot_heading, target_heading
    global prev_steering_angle, persistencia_obstaculo, obstaculo_clear_frames
    global ultimo_color_obstaculo, ultima_correccion_meta, bloqueo_giro_por_obstaculo
    global prioridad_obstaculo, turn_counter, race_finished

    robot_x = INITIAL_ROBOT_X
    robot_y = INITIAL_ROBOT_Y
    robot_heading = INITIAL_HEADING
    target_heading = INITIAL_TARGET_HEADING

    prev_steering_angle = 0.0
    persistencia_obstaculo = 0
    obstaculo_clear_frames = 0
    ultimo_color_obstaculo = None
    ultima_correccion_meta = 0.0
    bloqueo_giro_por_obstaculo = False
    prioridad_obstaculo = False
    turn_counter = 0
    race_finished = False

    pid_giro.prev_error = 0
    pid_vision.prev_error = 0
    pid_centro.prev_error = 0

# --- Físicas del Chasis RC ---
robot_x, robot_y = INITIAL_ROBOT_X, INITIAL_ROBOT_Y # Posición del eje trasero
robot_heading = INITIAL_HEADING            # Lectura del Giroscopio
target_heading = INITIAL_TARGET_HEADING

MAX_STEERING = STEERING_LIMIT             # Ángulo máximo del servo de dirección en grados
SLEW_RATE = STEERING_SLEW_RATE            # Cambio maximo del volante por frame

obstaculos = [] 

pid_vision = PID(0.15, 0.05)
# El Kp del giro ahora es más alto porque controla un servo limitado, no la rotación directa
pid_giro = PID(0.5, 0.15) 
pid_centro = PID(0.05, 0.01)

prev_steering_angle = 0.0
persistencia_obstaculo = 0
obstaculo_clear_frames = 0
ultimo_color_obstaculo = None
ultima_correccion_meta = 0.0
bloqueo_giro_por_obstaculo = False
prioridad_obstaculo = False
turn_counter = 0
race_finished = False
frame_counter = 0

font_debug = pygame.font.SysFont(DEBUG_FONT_NAME, DEBUG_FONT_SIZE)


def dibujar_debug(screen, lines, x, y, color=WHITE, line_gap=18):
    for line in lines:
        surface = font_debug.render(line, True, color)
        screen.blit(surface, (x, y))
        y += line_gap

def calcular_distancia_pared(rx, ry, angulo):
    rad = math.radians(angulo)
    ex = rx + math.cos(rad) * RAY_DISTANCE
    ey = ry - math.sin(rad) * RAY_DISTANCE
    dist_min = RAY_DISTANCE

    clip_ext = RECT_EXTERIOR.clipline(rx, ry, ex, ey)
    if clip_ext:
        d1 = math.hypot(clip_ext[0][0] - rx, clip_ext[0][1] - ry)
        d2 = math.hypot(clip_ext[1][0] - rx, clip_ext[1][1] - ry)
        dist_min = min(dist_min, max(d1, d2)) 

    clip_int = RECT_INTERIOR.clipline(rx, ry, ex, ey)
    if clip_int:
        d1 = math.hypot(clip_int[0][0] - rx, clip_int[0][1] - ry)
        d2 = math.hypot(clip_int[1][0] - rx, clip_int[1][1] - ry)
        dist_min = min(dist_min, min(d1, d2))

    return max(1, dist_min)

def camara_wro(rx, ry, heading, obstaculos):
    visibles = []
    # La cámara está montada en la parte frontal (eje delantero)
    fx = rx + WHEELBASE * math.cos(math.radians(heading))
    fy = ry - WHEELBASE * math.sin(math.radians(heading))
    
    for obs in obstaculos:
        dx, dy = obs[0] - fx, obs[1] - fy
        dist = math.hypot(dx, dy)
        if dist > VIEW_DISTANCE: 
            continue 
        
        angulo_abs = math.degrees(math.atan2(-dy, dx))
        dif_angulo = (angulo_abs - heading + 180) % 360 - 180
        
        if abs(dif_angulo) < CAMERA_FOV_DEG / 2: 
            dist_pared = calcular_distancia_pared(fx, fy, angulo_abs)
            if dist < dist_pared:
                pixel_x = ((dif_angulo + (CAMERA_FOV_DEG / 2)) / CAMERA_FOV_DEG) * PIXELS_PER_VIEW
                visibles.append((PIXELS_PER_VIEW - pixel_x, obs[2], dist))
            
    if visibles:
        visibles.sort(key=lambda v: v[2])
        return visibles[0]
    return None, None, None

running = True
while running:
    frame_counter += 1
    for event in pygame.event.get():
        if event.type == pygame.QUIT: 
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()
            if mx < 800: 
                if event.button == 1: obstaculos.append([mx, my, RED])
                elif event.button == 3: obstaculos.append([mx, my, GREEN])
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                reset_car_state()
            if event.key == pygame.K_x: 
                if obstaculos:
                    mx, my = pygame.mouse.get_pos()
                    obstaculos.sort(key=lambda o: math.hypot(o[0]-mx, o[1]-my))
                    obstaculos.pop(0)

    # El centro de la cámara para detectar paredes está en el frente
    front_x = robot_x + WHEELBASE * math.cos(math.radians(robot_heading))
    front_y = robot_y - WHEELBASE * math.sin(math.radians(robot_heading))

    error_alineacion = abs((robot_heading - target_heading + 180) % 360 - 180)

    # 2. Esquivar u Centrar
    pixel_obs, color_obs, dist_obs = camara_wro(robot_x, robot_y, robot_heading, obstaculos)
    correccion_meta = 0
    modo_conduccion = "centro"
    bloqueo_giro_por_obstaculo = False
    prioridad_obstaculo = False

    if pixel_obs is not None:
        persistencia_obstaculo = OBSTACLE_MEMORY_FRAMES
        obstaculo_clear_frames = 0
        ultimo_color_obstaculo = color_obs
        bloqueo_giro_por_obstaculo = True
        prioridad_obstaculo = True

        if color_obs == RED:
            objetivo_pixel = RED_PIXEL_DRAW_LINE
            error_pixel = objetivo_pixel - pixel_obs
            if abs(error_pixel) > OBSTACLE_HOLD_BAND:
                correccion_meta = pid_vision.calcular(error_pixel)
                ultima_correccion_meta = correccion_meta
                modo_conduccion = "evadiendo_rojo"
            else:
                pid_vision.prev_error = 0
                ultima_correccion_meta = 0
                
        elif color_obs == GREEN:
            objetivo_pixel = GREEN_PIXEL_DRAW_LINE
            error_pixel = objetivo_pixel - pixel_obs
            if abs(error_pixel) > OBSTACLE_HOLD_BAND:
                correccion_meta = pid_vision.calcular(error_pixel)
                ultima_correccion_meta = correccion_meta
                modo_conduccion = "evadiendo_verde"
            else:
                pid_vision.prev_error = 0
                ultima_correccion_meta = 0
                
        correccion_meta = clamp(correccion_meta, -VISION_MAX_CORRECTION, VISION_MAX_CORRECTION)
    elif persistencia_obstaculo > 0:
        persistencia_obstaculo -= 1
        obstaculo_clear_frames = 0
        bloqueo_giro_por_obstaculo = True
        prioridad_obstaculo = True
        correccion_meta = clamp(ultima_correccion_meta * OBSTACLE_FADE_FACTOR, -OBSTACLE_MAX_CORRECTION, OBSTACLE_MAX_CORRECTION)
        if ultimo_color_obstaculo == RED:
            modo_conduccion = "memoria_rojo"
        elif ultimo_color_obstaculo == GREEN:
            modo_conduccion = "memoria_verde"
    else:
        obstaculo_clear_frames += 1
        pid_vision.prev_error = 0 # Limpia la memoria del PID si no hay obstáculos
        
        if error_alineacion < CENTER_ALIGN_THRESHOLD:
            dist_pared_izq = calcular_distancia_pared(front_x, front_y, robot_heading + WALL_SIDE_ANGLE_LEFT)
            dist_pared_der = calcular_distancia_pared(front_x, front_y, robot_heading + WALL_SIDE_ANGLE_RIGHT)
            if dist_pared_izq < CENTER_WALL_THRESHOLD and dist_pared_der < CENTER_WALL_THRESHOLD:
                error_centro = dist_pared_izq - dist_pared_der
                correccion_meta = pid_centro.calcular(error_centro)
                correccion_meta = clamp(correccion_meta, -CENTER_MAX_CORRECTION, CENTER_MAX_CORRECTION)
                modo_conduccion = "centrando"

    # 1. Detección de Esquinas
    # No permitimos el giro de vuelta mientras el obstáculo siga visible o en memoria.
    if not race_finished and not prioridad_obstaculo and obstaculo_clear_frames >= OBSTACLE_CLEAR_FRAMES and error_alineacion < CORNER_ALIGN_THRESHOLD:
        dist_orilla_izq = calcular_distancia_pared(front_x, front_y, robot_heading + WALL_FRONT_ANGLE_LEFT)
        dist_orilla_der = calcular_distancia_pared(front_x, front_y, robot_heading + WALL_FRONT_ANGLE_RIGHT)

        if dist_orilla_izq > CORNER_WALL_THRESHOLD and dist_orilla_der < CORNER_WALL_THRESHOLD:
            target_heading += 90
            target_heading = (target_heading + 180) % 360 - 180
            turn_counter += 1
            if turn_counter >= 12:
                race_finished = True
        elif dist_orilla_der > CORNER_WALL_THRESHOLD and dist_orilla_izq < CORNER_WALL_THRESHOLD:
            target_heading -= 90
            target_heading = (target_heading + 180) % 360 - 180
            turn_counter += 1
            if turn_counter >= 12:
                race_finished = True
    # 3. CONTROL PID DEL SERVO DE DIRECCIÓN
    angulo_deseado_final = target_heading + correccion_meta
    error_giro = (angulo_deseado_final - robot_heading + 180) % 360 - 180 
    
    # El PID ya no gira el carro mágico, le dice al servo qué ángulo tomar
    raw_steering_angle = pid_giro.calcular(error_giro)
    
    # Límite mecánico de la dirección (ej. un servo real no gira 360 grados)
    steering_angle = clamp(raw_steering_angle, -MAX_STEERING, MAX_STEERING)
    delta_steering = steering_angle - prev_steering_angle
    if delta_steering > SLEW_RATE:
        steering_angle = prev_steering_angle + SLEW_RATE
    elif delta_steering < -SLEW_RATE:
        steering_angle = prev_steering_angle - SLEW_RATE
    prev_steering_angle = steering_angle

    # 4. CINEMÁTICA DE BICICLETA (Ackermann)
    rad_steering = math.radians(steering_angle)
    turn_intensity = min(1.0, abs(steering_angle) / MAX_STEERING)
    velocidad_actual = VELOCIDAD * (1.0 - (TURN_SPEED_FACTOR * turn_intensity))
    if prioridad_obstaculo:
        velocidad_actual *= OBSTACLE_PRIORITY_SPEED_FACTOR
    elif modo_conduccion != "centro":
        velocidad_actual *= AVOIDANCE_SPEED_FACTOR
    velocidad_actual = max(0.6, velocidad_actual)
    if race_finished:
        velocidad_actual = 0.0
    
    # El giroscopio real cambia dependiendo de la velocidad y el volante
    delta_heading = math.degrees((velocidad_actual / WHEELBASE) * math.tan(rad_steering))
    robot_heading += delta_heading
    robot_heading = (robot_heading + 180) % 360 - 180 
    
    # Avanzar el eje trasero
    rad_heading = math.radians(robot_heading)
    robot_x += velocidad_actual * math.cos(rad_heading)
    robot_y -= velocidad_actual * math.sin(rad_heading)

    if DEBUG and frame_counter % DEBUG_PRINT_EVERY == 0:
        print(
            f"[dbg] mode={modo_conduccion} obs={color_obs} pix={pixel_obs} dist={dist_obs} "
            f"mem={persistencia_obstaculo} prio={prioridad_obstaculo} target={target_heading:.1f} err={error_giro:.1f} "
            f"steer_raw={raw_steering_angle:.1f} steer={steering_angle:.1f} speed={velocidad_actual:.1f} heading={robot_heading:.1f}",
            flush=True,
        )

    # --- Renderizado Mapa ---
    screen.fill(GRAY)
    pygame.draw.rect(screen, WHITE, RECT_EXTERIOR)
    pygame.draw.rect(screen, BLACK, RECT_EXTERIOR, 8) 
    pygame.draw.rect(screen, GRAY, RECT_INTERIOR)
    pygame.draw.rect(screen, BLACK, RECT_INTERIOR, 8) 

    for obs in obstaculos:
        pygame.draw.rect(screen, obs[2], (obs[0]-10, obs[1]-10, 20, 20))
    
    # Dibujar Chasis Realista
    front_x_draw = robot_x + WHEELBASE * math.cos(rad_heading)
    front_y_draw = robot_y - WHEELBASE * math.sin(rad_heading)
    
    # Línea del chasis
    pygame.draw.line(screen, BLACK, (robot_x, robot_y), (front_x_draw, front_y_draw), 4)
    # Llantas traseras fijas
    pygame.draw.circle(screen, BLUE, (int(robot_x), int(robot_y)), 6)
    # Llantas delanteras (Muestran visualmente el steering_angle del servo)
    rad_llanta_frente = math.radians(robot_heading + steering_angle)
    wx1 = front_x_draw + 8 * math.cos(rad_llanta_frente)
    wy1 = front_y_draw - 8 * math.sin(rad_llanta_frente)
    wx2 = front_x_draw - 8 * math.cos(rad_llanta_frente)
    wy2 = front_y_draw + 8 * math.sin(rad_llanta_frente)
    pygame.draw.line(screen, RED, (wx1, wy1), (wx2, wy2), 6)

    # --- Renderizado POV Raycasting 3D ---
    pygame.draw.rect(screen, DARK_GRAY, (MAP_X0, 0, PANEL_WIDTH, POV_HEIGHT)) 
    pygame.draw.rect(screen, WHITE, (MAP_X0, POV_HEIGHT, PANEL_WIDTH, POV_HEIGHT))   
    
    ANCHO_RAYO = PANEL_WIDTH // RAYS

    for i in range(RAYS):
        ray_ang = robot_heading + (CAMERA_FOV_DEG / 2) - (i * (CAMERA_FOV_DEG / RAYS))
        dist = calcular_distancia_pared(front_x_draw, front_y_draw, ray_ang)
        dist_corregida = max(1, dist * math.cos(math.radians(robot_heading - ray_ang)))
        
        altura_pared = min(POV_HEIGHT, (POV_HEIGHT * RAY_WALL_SCALE) / dist_corregida)
        pygame.draw.rect(screen, BLACK, (MAP_X0 + i * ANCHO_RAYO, POV_HEIGHT - (altura_pared / 2), ANCHO_RAYO, altura_pared))

    pygame.draw.line(screen, GREEN, (MAP_X0 + (GREEN_PIXEL_DRAW_LINE / PIXELS_PER_VIEW) * PANEL_WIDTH, 0), (MAP_X0 + (GREEN_PIXEL_DRAW_LINE / PIXELS_PER_VIEW) * PANEL_WIDTH, PANEL_HEIGHT), 1)
    pygame.draw.line(screen, RED, (MAP_X0 + (RED_PIXEL_DRAW_LINE / PIXELS_PER_VIEW) * PANEL_WIDTH, 0), (MAP_X0 + (RED_PIXEL_DRAW_LINE / PIXELS_PER_VIEW) * PANEL_WIDTH, PANEL_HEIGHT), 1)

    if pixel_obs is not None:
        pov_x = MAP_X0 + (pixel_obs / PIXELS_PER_VIEW) * PANEL_WIDTH
        size = int(max(PADDING, 8000 / dist_obs))
        pygame.draw.rect(screen, color_obs, (pov_x - size/2, POV_HEIGHT - size/2, size, size))

    debug_panel = pygame.Surface((PANEL_WIDTH, 390), pygame.SRCALPHA)
    debug_panel.fill((0, 0, 0, DEBUG_PANEL_ALPHA))
    screen.blit(debug_panel, (MAP_X0, DEBUG_PANEL_Y))
    lap_display = f"TERMINADO" if race_finished else f"{turn_counter // 4}/3"
    dibujar_debug(
        screen,
        [
            f"mode: {modo_conduccion}",
            f"obs: {color_obs} pix={pixel_obs} dist={dist_obs}",
            f"memory: {persistencia_obstaculo} last={ultimo_color_obstaculo}",
            f"target: {target_heading:.1f}  align: {error_alineacion:.1f}",
            f"err: {error_giro:.1f}  steer: {steering_angle:.1f}",
            f"raw: {raw_steering_angle:.1f}  corr: {correccion_meta:.1f}",
            f"vueltas: {lap_display}  giros: {turn_counter}/12",
        ],
        MAP_X0 + PADDING + 5,
        DEBUG_PANEL_Y + PADDING + 10,
        WHITE,
        20,
    )

    if race_finished:
        font_big = pygame.font.SysFont(DEBUG_FONT_NAME, 48, bold=True)
        txt = font_big.render("TERMINADO - 3 VUELTAS", True, (0, 255, 0))
        screen.blit(txt, (MAP_X0 + PANEL_WIDTH // 2 - txt.get_width() // 2, HEIGHT // 2 - 24))

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()