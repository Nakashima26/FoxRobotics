"""
Simulador de escritorio para Pure Pursuit — WRO Future Engineers.

IMPORTANTE — qué es real y qué es modelo:

  REAL (importado sin modificar):
    - config.py         → todos tus parámetros actuales
    - centerline.py      → detect_centerline(), draw_bev_debug() TAL CUAL
    - controller.py      → PurePursuitController.compute() TAL CUAL
    - obstacle_memory.py → ObstacleMemory TAL CUAL (opcional, --memoria)

  MODELO (reimplementado en Python porque el ESP32 corre C++, no se puede
  importar):
    - FSM SIGUIENDO/GIRANDO
    - Lectura de ultrasónicos (simulada con geometría del "mundo")
    - ppSteerGain / ppServoGain / detectarEsquina() / gating prio+mem+heading
    - Modelo cinemático de bicicleta (integra la pose del robot)

  El "mundo" (pista + latas) tampoco es parte de tu robot real — aquí
  reemplaza a la cámara física: en vez de una imagen de cámara + homografía,
  genera directamente una vista BEV geométricamente correcta (sin distorsión
  de perspectiva) para la pose actual del robot.  Es una verdad de terreno
  ideal — más limpia que tu cámara real, así que si algo falla aquí, muy
  probablemente también falla (o falla peor) en el carro físico.

USO:
  cd src/RASPI/cam/              (junto a config.py, centerline.py, etc.)
  python -m pure_pursuit.pure_pursuit_sim
  python -m pure_pursuit.pure_pursuit_sim --save-gif salida.gif --steps 260
  python -m pure_pursuit.pure_pursuit_sim --ppservogain 1.8 --wheelbase-mm 120

CONTROLES (ventana interactiva):
  ESC / cerrar ventana : salir
  SPACE                : pausar/reanudar
"""

import argparse
import math
import sys

import cv2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from . import config as C
from .centerline import detect_centerline, draw_bev_debug, map_obstacle_to_bev
from .controller import PurePursuitController
from .obstacle_memory import ObstacleMemory


# ═══════════════════════════════════════════════════════════════════════════
# Definición del "mundo" (pista + obstáculos) — TODO EN mm
# ═══════════════════════════════════════════════════════════════════════════

# Pista de ejemplo: recta -> esquina 90° a la derecha -> recta, con una lata
# roja antes de la esquina y una verde después (como en tus capturas).
TRACK_CLOSED = True   # loop cerrado (pista real WRO: vueltas alrededor de la isla)

# Pista 3m x 3m, isla central 1m x 1m → centerline del pasillo a ±1000mm del
# centro (a medio camino entre la pared exterior en ±1500mm y la isla en ±500mm)
TRACK_WAYPOINTS_MM = [
    (1000, -1000),
    (1000,  1000),
    (-1000, 1000),
    (-1000,-1000),
]
CORRIDOR_WIDTH_MM = 600.0   # deja ~200mm de margen a cada pared con este centerline

OBSTACLES_MM = [
    # (x_mm, y_mm, color, radio_mm) — edítalos según el reto que quieras probar
    (1000,    0, "Red",   32.5),   # a media recta del lado derecho
    (   0, 1000, "Green", 32.5),   # a media recta del lado superior
    (1100, 500, "Red",   32.5),   # a media recta del lado derecho
    (500, 1100, "Green", 32.5),   # a media recta del lado superior
]

ROBOT_START = dict(x_mm=1000.0, y_mm=-900.0, heading_deg=0.0)   # arranca en la recta derecha


# ═══════════════════════════════════════════════════════════════════════════
# Geometría del mundo → vista BEV robot-relativa (afín, sin distorsión)
# ═══════════════════════════════════════════════════════════════════════════

FLOOR_COLOR_BGR = (200, 200, 200)   # cae dentro de FLOOR_LOWER/UPPER de config.py
WALL_COLOR_BGR  = (0, 0, 0)         # V=0 → nunca cae en el rango de piso


def world_to_bev(px_mm, py_mm, robot_x, robot_y, heading_deg):
    """Misma convención que BEVTransformer._real_mm_to_bev_px, pero relativa
    a la pose ACTUAL del robot en vez de a los 4 puntos fijos de calibración."""
    h = math.radians(heading_deg)
    dx = px_mm - robot_x
    dy = py_mm - robot_y
    forward = dx * math.sin(h) + dy * math.cos(h)   # + = adelante del robot
    lateral = dx * math.cos(h) - dy * math.sin(h)   # + = derecha del robot
    bev_x = C.ROBOT_BEV_X + lateral / C.MM_PER_PX
    bev_y = C.ROBOT_BEV_Y - forward / C.MM_PER_PX
    return bev_x, bev_y


def offset_polyline(points_mm, offset_mm, closed=False):
    """Pared desplazada perpendicular al centerline — útil para pistas con
    tramos rectos entre esquinas.  NO usar en polígonos donde CADA vértice
    es una esquina de 90° (como un cuadrado puro): ahí el promedio de
    direcciones vecinas en cada vértice no tiene ningún tramo recto de
    referencia y produce un offset diagonal incorrecto.  Para la pista
    cuadrada de abajo, las paredes se definen explícitamente en su lugar."""
    pts = np.array(points_mm, dtype=float)
    n = len(pts)
    out = np.zeros_like(pts)
    for i in range(n):
        if closed:
            prev_pt = pts[(i - 1) % n]
            next_pt = pts[(i + 1) % n]
            d = next_pt - prev_pt
        elif i == 0:
            d = pts[1] - pts[0]
        elif i == n - 1:
            d = pts[-1] - pts[-2]
        else:
            d = pts[i + 1] - pts[i - 1]
        d = d / (np.linalg.norm(d) + 1e-9)
        normal = np.array([d[1], -d[0]])   # perpendicular
        out[i] = pts[i] + normal * offset_mm
    return out


def _square_corners(half_side_mm):
    return np.array([
        [ half_side_mm, -half_side_mm],
        [ half_side_mm,  half_side_mm],
        [-half_side_mm,  half_side_mm],
        [-half_side_mm, -half_side_mm],
    ], dtype=float)


# Paredes REALES de la pista (no derivadas del centerline): pared exterior de
# 3m x 3m y pared de la isla central de 1m x 1m — geometría exacta, sin
# depender de offset_polyline.
OUTER_WALL_MM  = _square_corners(1500.0)
ISLAND_WALL_MM = _square_corners(500.0)
WALL_POLYGONS  = [OUTER_WALL_MM, ISLAND_WALL_MM]

# Se conservan por compatibilidad con el render del mundo (dibujo de
# referencia) — para ESTA pista, usa las paredes reales de arriba, no estas.
LEFT_WALL_MM  = ISLAND_WALL_MM
RIGHT_WALL_MM = OUTER_WALL_MM


def render_bev(robot_x, robot_y, heading_deg):
    """Genera una imagen BEV_W x BEV_H geométricamente correcta del pasillo,
    vista desde la pose actual del robot — reemplaza cámara + homografía."""
    canvas = np.full((C.BEV_H, C.BEV_W, 3), WALL_COLOR_BGR, dtype=np.uint8)

    pts_bev = np.array(
        [world_to_bev(x, y, robot_x, robot_y, heading_deg) for x, y in TRACK_WAYPOINTS_MM],
        dtype=np.int32,
    )
    thickness_px = max(2, int(round(CORRIDOR_WIDTH_MM / C.MM_PER_PX)))
    cv2.polylines(canvas, [pts_bev], isClosed=TRACK_CLOSED,
                  color=FLOOR_COLOR_BGR, thickness=thickness_px, lineType=cv2.LINE_8)
    return canvas


def obstacles_to_bev(robot_x, robot_y, heading_deg):
    """Proyecta las latas del mundo a coords BEV — igual que map_obstacle_to_bev
    pero sin cámara de por medio.  Respeta bev_in_bounds como el sistema real."""
    out = []
    for ox, oy, color, _radius in OBSTACLES_MM:
        bx, by = world_to_bev(ox, oy, robot_x, robot_y, heading_deg)
        if 0.0 <= bx < C.BEV_W and 0.0 <= by < C.BEV_H:
            out.append((bx, by, color))
    return out


def ray_segment_intersect(origin, direction, seg_a, seg_b):
    v1 = origin - seg_a
    v2 = seg_b - seg_a
    v3 = np.array([-direction[1], direction[0]])
    denom = np.dot(v2, v3)
    if abs(denom) < 1e-9:
        return None
    t1 = np.cross(v2, v1) / denom
    t2 = np.dot(v1, v3) / denom
    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return t1
    return None


def ultrasonic_distances_cm(robot_x, robot_y, heading_deg, max_cm=200.0):
    """Simula HC-SR04 izq/der lanzando un rayo lateral y viendo dónde pega
    la pared más cercana — la exterior o la de la isla, la que esté más
    cerca en esa dirección — el mismo efecto físico que produce las lecturas
    reales, incluyendo la 'apertura' cuando el pasillo dobla."""
    h = math.radians(heading_deg)
    origin = np.array([robot_x, robot_y])
    right_dir = np.array([math.cos(h), -math.sin(h)])
    left_dir  = -right_dir

    def cast(direction):
        best = None
        for poly in WALL_POLYGONS:
            n = len(poly)
            for i in range(n):
                a, b = poly[i], poly[(i + 1) % n]   # polígonos siempre cerrados
                t = ray_segment_intersect(origin, direction, a, b)
                if t is not None and (best is None or t < best):
                    best = t
        return min(max_cm, (best / 10.0) if best is not None else max_cm)  # mm→cm

    dist_r = cast(right_dir)
    dist_l = cast(left_dir)
    return dist_l, dist_r


# ═══════════════════════════════════════════════════════════════════════════
# MODELO del firmware ESP32 (reimplementado en Python — no es el .ino real)
# ═══════════════════════════════════════════════════════════════════════════

class ESP32Model:
    def __init__(self, ppservogain: float, wheelbase_mm: float,
                 heading_settle_deg: float, esquina_debounce: int,
                 wall_settle_cm: float, recuperando_timeout_s: float = 1.5,
                 kp_wall: float = 1.0, kd_wall: float = 1.2,
                 kp_gyro: float = 2.0, kd_gyro: float = 0.5):
        self.estado = "SIGUIENDO"
        self.ppSteerGain = C.PP_STEER_GAIN
        self.ppServoGain = ppservogain
        self.wheelbase_mm = wheelbase_mm     # batalla REAL usada para mover el robot
        self.centro_servo = 80
        self.angulo_gyro = 0.0
        self.angulo_objetivo = 0.0
        self.direccion_izq = True
        self.primer_giro = False
        self.ang_giro = 75.0
        self.contador_esquina = 0
        self.esquina_debounce = esquina_debounce
        self.umbral_pared_cm = 100.0
        self.heading_settle_deg = heading_settle_deg
        self.wall_settle_cm = wall_settle_cm
        self.recuperando_timeout_s = recuperando_timeout_s
        self.recuperando_entry_t = None
        self.last_turn_t = -999.0
        self.cooldown_giro_s = 1.5
        self.turns_completed = 0
        self.prio_anterior = False

        # PID de pared / gyro — mismas ganancias que el .ino, usadas en RECUPERANDO
        self.kp_wall, self.kd_wall = kp_wall, kd_wall
        self.kp_gyro, self.kd_gyro = kp_gyro, kd_gyro
        self.prev_error_wall = 0.0
        self.prev_error_gyro = 0.0

    def error_gyro(self):
        return self.angulo_objetivo - self.angulo_gyro

    def detectar_esquina(self, dist_l_cm, dist_r_cm):
        apertura = (dist_l_cm > self.umbral_pared_cm) or (dist_r_cm > self.umbral_pared_cm)
        self.contador_esquina = self.contador_esquina + 1 if apertura else 0
        return self.contador_esquina >= self.esquina_debounce

    def _servo_offset_to_wheel_deg(self, servo_offset_deg):
        """Convierte un offset de servo (grados sumados/restados de centro_servo)
        a grados de llanta reales, usando la misma calibración ppServoGain."""
        servo_angle = self.centro_servo + servo_offset_deg
        servo_angle = max(20, min(150, servo_angle))
        return (self.centro_servo - servo_angle) / self.ppServoGain

    def _wall_gyro_output(self, dt_s, dist_l_cm, dist_r_cm):
        """Réplica de las dos PID que YA existen en controlPID() del .ino —
        aquí sí se usan (en SIGUIENDO real se calculan pero se descartan)."""
        error_wall = max(-50.0, min(50.0, dist_l_cm - dist_r_cm))
        deriv_wall = (error_wall - self.prev_error_wall) / max(dt_s, 1e-3)
        output_wall = self.kp_wall * error_wall + self.kd_wall * deriv_wall
        self.prev_error_wall = error_wall

        error_gyro = max(-20.0, min(20.0, self.error_gyro()))
        deriv_gyro = (error_gyro - self.prev_error_gyro) / max(dt_s, 1e-3)
        output_gyro = self.kp_gyro * error_gyro + self.kd_gyro * deriv_gyro
        self.prev_error_gyro = error_gyro

        return error_wall, output_wall, output_gyro

    def step(self, t_s, dt_s, dist_l_cm, dist_r_cm, obs_norm, prio, mem):
        # NOTA: angulo_gyro se actualiza DESPUÉS de este step (en el loop
        # principal), con el delta de heading real que produjo el wheel_deg
        # de este mismo frame — igual que en el carro real, el gyro reporta
        # el movimiento ya ocurrido, con un frame de retraso natural.

        if self.estado == "RECUPERANDO":
            error_wall, output_wall, output_gyro = self._wall_gyro_output(
                dt_s, dist_l_cm, dist_r_cm)
            output_final = max(-25.0, min(25.0, output_wall + output_gyro))

            wall_ok = abs(error_wall) < self.wall_settle_cm
            heading_ok = abs(self.error_gyro()) < self.heading_settle_deg
            timed_out = (self.recuperando_entry_t is not None
                         and (t_s - self.recuperando_entry_t) > self.recuperando_timeout_s)
            if prio:
                self.estado = "SIGUIENDO"          # reapareció algo → vuelve a esquivar
            elif (wall_ok and heading_ok) or timed_out:
                # timed_out: red de seguridad — sin esto, si el robot entra a
                # RECUPERANDO cerca de una esquina donde un lado lee "sin
                # pared" legítimamente, wall_ok puede no cumplirse NUNCA y el
                # estado se queda atorado para siempre (lo vimos en la sim).
                self.estado = "SIGUIENDO"
            self.prio_anterior = prio
            return self._servo_offset_to_wheel_deg(output_final)

        if self.estado == "SIGUIENDO":
            # Flanco de bajada: la cámara ACABA de dejar de ver el obstáculo
            if self.prio_anterior and not prio:
                self.estado = "RECUPERANDO"
                self.recuperando_entry_t = t_s
                self.prio_anterior = prio
                error_wall, output_wall, output_gyro = self._wall_gyro_output(
                    dt_s, dist_l_cm, dist_r_cm)
                output_final = max(-25.0, min(25.0, output_wall + output_gyro))
                return self._servo_offset_to_wheel_deg(output_final)
            self.prio_anterior = prio

            bloqueado = prio or (mem > 0)   # ya no depende de heading — eso lo cubre RECUPERANDO

            corner = self.detectar_esquina(dist_l_cm, dist_r_cm)
            if (t_s - self.last_turn_t > self.cooldown_giro_s and not bloqueado
                    and corner and t_s > 1.0):
                self.estado = "GIRANDO"
                self.angulo_gyro = 0.0
                if not self.primer_giro:
                    self.direccion_izq = dist_l_cm > dist_r_cm
                    self.primer_giro = True
                return 0.0   # servo centrado durante la transición de este frame

            steer_deg = max(-self.ppSteerGain, min(self.ppSteerGain, obs_norm * self.ppSteerGain))
            servo_angle = self.centro_servo - steer_deg * self.ppServoGain
            servo_angle = max(20, min(150, servo_angle))
            wheel_deg = (self.centro_servo - servo_angle) / self.ppServoGain
            return wheel_deg

        else:  # GIRANDO — ángulo de servo fijo, igual que el .ino
            delta = abs(self.angulo_gyro)
            # ino: servo=150 → izquierda, servo=20 → derecha (centro=80).
            # Aquí "wheel_deg" positivo = derecha (misma convención que steer_deg
            # de la Pi), así que izquierda = wheel_deg negativo.
            wheel_deg_max = (self.centro_servo - 20) / self.ppServoGain
            wheel_deg = -wheel_deg_max if self.direccion_izq else wheel_deg_max

            if delta >= self.ang_giro:
                self.angulo_objetivo = self.angulo_gyro
                self.last_turn_t = t_s
                self.estado = "SIGUIENDO"
                self.turns_completed += 1
            return wheel_deg


# ═══════════════════════════════════════════════════════════════════════════
# Loop de simulación
# ═══════════════════════════════════════════════════════════════════════════

def run(args):
    controller = PurePursuitController()
    memory = ObstacleMemory() if args.memoria else None
    esp = ESP32Model(
        ppservogain=args.ppservogain,
        wheelbase_mm=args.wheelbase_mm,
        heading_settle_deg=args.heading_settle_deg,
        esquina_debounce=args.esquina_debounce,
        wall_settle_cm=args.wall_settle_cm,
        recuperando_timeout_s=args.recuperando_timeout_s,
    )

    robot_x, robot_y, heading = (ROBOT_START["x_mm"], ROBOT_START["y_mm"],
                                  ROBOT_START["heading_deg"])
    speed_mms = args.speed_mms
    dt = args.dt

    trail = []           # trayectoria real del robot (mundo, mm)
    planned_trail = []    # último punto look-ahead planeado (mundo, mm)

    matplotlib.use("Agg" if args.save_gif else matplotlib.get_backend())
    fig, (ax_world, ax_bev) = plt.subplots(1, 2, figsize=(12, 6))
    frames_for_gif = []

    t = 0.0
    for step_i in range(args.steps):
        bev_img = render_bev(robot_x, robot_y, heading)
        new_obs = obstacles_to_bev(robot_x, robot_y, heading)

        if memory is not None:
            bev_obstacles = memory.update(new_obs, dt, heading)
        else:
            bev_obstacles = new_obs

        path_points = detect_centerline(bev_img, bev_obstacles)

        steer_deg = 0.0
        lookahead_pt = (float(C.ROBOT_BEV_X), float(C.ROBOT_BEV_Y))
        pp_active = False
        obs_norm = 0.0
        if len(path_points) >= C.MIN_PATH_PTS:
            steer_deg, lookahead_pt = controller.compute(
                path_points, C.ROBOT_BEV_X, C.ROBOT_BEV_Y)
            obs_norm = controller.normalize(steer_deg)
            pp_active = True

        dist_l_cm, dist_r_cm = ultrasonic_distances_cm(robot_x, robot_y, heading)
        prio = len(bev_obstacles) > 0
        mem_n = len(bev_obstacles) if memory is not None else 0

        wheel_deg = esp.step(t, dt, dist_l_cm, dist_r_cm, obs_norm, prio, mem_n)

        # ── Modelo cinemático de bicicleta ──────────────────────────────────
        wheel_rad = math.radians(max(-35.0, min(35.0, wheel_deg)))
        dheading_deg = math.degrees(speed_mms * dt / max(1.0, esp.wheelbase_mm)
                                     * math.tan(wheel_rad))
        esp.angulo_gyro += dheading_deg   # el "gyro" reporta el giro que este wheel_deg produjo
        heading += dheading_deg
        h = math.radians(heading)
        robot_x += speed_mms * dt * math.sin(h)
        robot_y += speed_mms * dt * math.cos(h)
        t += dt

        trail.append((robot_x, robot_y))
        if pp_active:
            lx_mm = (lookahead_pt[0] - C.ROBOT_BEV_X) * C.MM_PER_PX
            ly_mm = (C.ROBOT_BEV_Y - lookahead_pt[1]) * C.MM_PER_PX
            hh = math.radians(heading)
            wx = robot_x + lx_mm * math.cos(hh) + ly_mm * math.sin(hh)
            wy = robot_y - lx_mm * math.sin(hh) + ly_mm * math.cos(hh)
            planned_trail.append((wx, wy))

        if args.print_log:
            print(f"t={t:5.2f}s estado={esp.estado:10s} obs={obs_norm:+.3f} "
                  f"wheel={wheel_deg:+5.1f} dist_L={dist_l_cm:5.1f} dist_R={dist_r_cm:5.1f} "
                  f"prio={int(prio)} mem={mem_n} pts={len(path_points)}", flush=True)

        if step_i % args.render_every == 0 or step_i == args.steps - 1:
            ax_world.clear(); ax_bev.clear()

            def _closed(arr):
                return np.vstack([arr, arr[0]]) if TRACK_CLOSED else arr

            wp = np.array(TRACK_WAYPOINTS_MM)
            wp_plot = _closed(wp)
            lw_plot = _closed(LEFT_WALL_MM)
            rw_plot = _closed(RIGHT_WALL_MM)
            ax_world.plot(wp_plot[:, 0], wp_plot[:, 1], "k--", lw=1, alpha=0.4)
            ax_world.plot(lw_plot[:, 0], lw_plot[:, 1], "k-", lw=2)
            ax_world.plot(rw_plot[:, 0], rw_plot[:, 1], "k-", lw=2)
            for ox, oy, color, _r in OBSTACLES_MM:
                c = "red" if color == "Red" else "green"
                ax_world.add_patch(plt.Circle((ox, oy), 32.5, color=c))
            tr = np.array(trail)
            ax_world.plot(tr[:, 0], tr[:, 1], "b-", lw=2, label="trayectoria real")
            if planned_trail:
                pt = np.array(planned_trail)
                ax_world.plot(pt[:, 0], pt[:, 1], "c.", ms=3, alpha=0.5, label="look-ahead planeado")
            ax_world.plot(robot_x, robot_y, "o", color="orange", ms=10)
            ax_world.arrow(robot_x, robot_y,
                            80 * math.sin(math.radians(heading)),
                            80 * math.cos(math.radians(heading)),
                            head_width=25, color="orange")
            ax_world.set_aspect("equal")
            ax_world.set_title(f"Mundo  t={t:.2f}s  estado={esp.estado}")
            ax_world.legend(loc="upper left", fontsize=7)

            bev_debug = draw_bev_debug(bev_img, path_points, lookahead_pt,
                                        bev_obstacles, steer_deg, pp_active)
            ax_bev.imshow(cv2.cvtColor(bev_debug, cv2.COLOR_BGR2RGB))
            ax_bev.set_title(f"BEV robot  obs={obs_norm:+.3f}  wheel={wheel_deg:+.1f}°")
            ax_bev.axis("off")

            plt.tight_layout()
            if args.save_gif:
                fig.canvas.draw()
                frame = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
                frame = frame.reshape(fig.canvas.get_width_height()[::-1] + (4,))
                frames_for_gif.append(frame[:, :, :3].copy())
            else:
                plt.pause(0.001)

        if esp.turns_completed >= args.max_turns:
            print(f"[SIM] {args.max_turns} giros completados en t={t:.2f}s", flush=True)
            break

    if args.save_gif:
        import imageio.v2 as imageio
        imageio.mimsave(args.save_gif, frames_for_gif, duration=args.dt * args.render_every)
        print(f"[SIM] GIF guardado en {args.save_gif}", flush=True)
    else:
        plt.show()


def parse_args():
    p = argparse.ArgumentParser(description="Simulador Pure Pursuit (usa tu código real)")
    p.add_argument("--speed-mms", type=float, default=200.0)
    p.add_argument("--dt", type=float, default=0.05)
    p.add_argument("--steps", type=int, default=2000)
    p.add_argument("--render-every", type=int, default=4)
    p.add_argument("--max-turns", type=int, default=12,
                    help="corta la sim al completar N giros (como TURNS_PER_RACE del .ino)")
    p.add_argument("--ppservogain", type=float, default=2.5,
                    help="mismo valor que ppServoGain en el .ino — ajústalo aquí para ver el efecto")
    p.add_argument("--wheelbase-mm", type=float, default=100.0,
                    help="batalla REAL del chasis en mm (mide la tuya)")
    p.add_argument("--heading-settle-deg", type=float, default=8.0)
    p.add_argument("--esquina-debounce", type=int, default=1,
                    help="usa 1 para reproducir el bug original, 4+ para el fix propuesto")
    p.add_argument("--wall-settle-cm", type=float, default=8.0,
                    help="qué tan parejo debe quedar dist_L/dist_R para considerar 'ya enderezado'")
    p.add_argument("--recuperando-timeout-s", type=float, default=1.5,
                    help="red de seguridad: fuerza salida de RECUPERANDO tras N segundos aunque wallOk nunca se cumpla")
    p.add_argument("--memoria", action="store_true", help="usa ObstacleMemory real")
    p.add_argument("--print-log", action="store_true")
    p.add_argument("--save-gif", type=str, default=None)
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())