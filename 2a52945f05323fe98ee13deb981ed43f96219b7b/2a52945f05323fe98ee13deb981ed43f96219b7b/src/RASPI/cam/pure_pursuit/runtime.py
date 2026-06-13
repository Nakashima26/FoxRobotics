"""
Runtime Pure Pursuit — WRO Future Engineers.

Punto de entrada principal del módulo.  Reemplaza wro_runtime.py cuando
la calibración BEV está disponible.

Pipeline por frame:
  1. Captura (ThreadedFrameGrabber, mismo que wro_runtime.py)
  2. vision.process_frame() → detección HSV de obstáculos rojo/verde
  3. Si BEV calibrado y sin prioridad de obstáculo:
       bev.warp() → detect_centerline() → PurePursuitController.compute()
       → mensaje V2 con pp=1
  4. Si obstáculo con prioridad O sin calibración BEV:
       PID de pixel (idéntico a wro_runtime.py) → mensaje V2 con pp=0
  5. TurnDetector (análisis de madera en ROI superior) → turn_hint
  6. SerialLink.send_line() → ESP32

Protocolo serial V2:
  V2,obs=+0.350,turn=0,state=pp_follow,prio=0,mem=0,pp=1
  V2,obs=-0.125,turn=0,state=avoid_red,prio=1,mem=18,pp=0

  • pp=1 → ESP32 usa ppSteerGain (35°) y suspende PID de paredes
  • pp=0 → ESP32 usa visionSteerGain (80) con PID de paredes activo (V1)

Secuencia de arranque (idéntica a wro_runtime.py):
  GPIO17 (botón) → LED GPIO27 → warmup cámara → UART READY → loop
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

# ── Importar infraestructura compartida de cam/ ──────────────────────────────
_CAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from vision import Vision
from wro_runtime import (
    ThreadedFrameGrabber,
    AsyncVideoWriter,
    SerialLink,
    TurnDetector,
    PID,
    next_output_path,
    resolve_output_path,
    CAM_FRAME_PATH,
)

from .bev import BEVTransformer
from .centerline import detect_centerline, map_obstacle_to_bev, draw_bev_debug
from .controller import PurePursuitController
from . import config as C


# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class PPConfig:
    # Cámara / captura
    cam_index:        int   = C.CAM_INDEX
    serial_port:      str   = C.SERIAL_PORT
    baudrate:         int   = C.BAUDRATE
    process_every_n:  int   = C.PROCESS_EVERY
    warmup_frames:    int   = C.WARMUP_FRAMES
    threaded_capture: bool  = True
    show_window:      bool  = True

    # Grabación
    record_orillas:  bool        = False
    record_output:   str | None  = None
    record_every_n:  int         = 6
    record_fps:      float       = 5.0

    # BEV
    calib_path: Path | None = None

    # Fallback PID de pixel (mismo que wro_runtime.py)
    red_target_px:           int   = C.RED_TARGET_PX
    green_target_px:         int   = C.GREEN_TARGET_PX
    pid_kp:                  float = C.PID_KP
    pid_kd:                  float = C.PID_KD
    correction_limit_px:     float = C.CORR_LIMIT_PX
    obstacle_hold_band:      float = 24.0
    obstacle_memory_frames:  int   = 18
    obstacle_clear_frames:   int   = 10
    obstacle_fade_factor:    float = 0.85
    obstacle_max_corr_px:    float = 35.0

    # TurnDetector
    turn_top_ratio:  float = 0.30
    turn_drop_ratio: float = 0.85

    neutral_x: int = 700


# ─────────────────────────────────────────────────────────────────────────────

class PPRuntime:
    """
    Runtime integrado con pipeline Pure Pursuit + fallback pixel PID.
    La infraestructura de captura, serial y GPIO es idéntica a wro_runtime.py.
    """

    def __init__(self, cfg: PPConfig):
        self.cfg = cfg

        # Visión
        self.vision    = Vision(cfg.cam_index)
        self.bev       = BEVTransformer(cfg.calib_path)
        self.controller = PurePursuitController()
        self.turn_detector = TurnDetector(cfg.turn_top_ratio, cfg.turn_drop_ratio)

        # PID de fallback (pixel-based)
        self.pid_obs = PID(cfg.pid_kp, cfg.pid_kd)

        # Serial
        self.serial_link = SerialLink(cfg.serial_port, cfg.baudrate)

        # Estado de obstáculo (espejo exacto de wro_runtime.py)
        self.persistencia_obstaculo  = 0
        self.obstaculo_clear_frames  = 0
        self.ultimo_color_obstaculo  = None
        self.ultima_correccion_px    = 0.0

        # Captura / grabación
        self.loop_count   = 0
        self.frame_grabber = None
        self.video_writer  = None
        self.record_count  = 0
        self.output_file   = (resolve_output_path(cfg.record_output)
                              if cfg.record_orillas else None)

        try:
            self.vision.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        if self.bev.is_calibrated:
            print("[PP] BEV calibrado — Pure Pursuit activo.", flush=True)
        else:
            print("[PP] Sin calibración BEV — usando fallback pixel PID.", flush=True)
            print(f"[PP] Corre: python -m pure_pursuit.calibrate", flush=True)

    # ── Detección de obstáculos (idéntico a wro_runtime.py) ──────────────────

    @staticmethod
    def _largest_object(objects):
        if not objects:
            return None, None
        obj = max(objects, key=lambda o: o[2] * o[3])
        x, y, w, h = obj
        return obj, x + w // 2

    def _obstacle_decision(self, positions: dict, frame_width: int) -> dict:
        red_obj,   red_x   = self._largest_object(positions.get("Red",   []))
        green_obj, green_x = self._largest_object(positions.get("Green", []))

        if red_obj and green_obj:
            use_red = red_obj[2] * red_obj[3] >= green_obj[2] * green_obj[3]
        else:
            use_red = bool(red_obj)

        if use_red and red_obj:
            x        = red_x
            error_px = float(self.cfg.red_target_px - x)
            color, mode = "Red", "avoid_red"
        elif green_obj:
            x        = green_x
            error_px = float(self.cfg.green_target_px - x)
            color, mode = "Green", "avoid_green"
        else:
            return {"detected": False, "color": "None", "x": None,
                    "error_px": 0.0, "vision_error_norm": 0.0,
                    "mode": "no_obstacle", "priority": False, "memory_frames": 0}

        if abs(error_px) > self.cfg.obstacle_hold_band:
            corr = self.pid_obs.compute(error_px)
            corr = max(-self.cfg.correction_limit_px,
                       min(self.cfg.correction_limit_px, corr))
            self.ultima_correccion_px = corr
        else:
            corr = 0.0
            self.pid_obs.prev_error = 0.0
            self.ultima_correccion_px = 0.0

        self.persistencia_obstaculo = self.cfg.obstacle_memory_frames
        self.obstaculo_clear_frames = 0
        self.ultimo_color_obstaculo = color

        norm = -(corr / max(1.0, self.cfg.correction_limit_px))
        return {"detected": True, "color": color, "x": x,
                "error_px": error_px, "vision_error_norm": float(norm),
                "mode": mode, "priority": True,
                "memory_frames": self.persistencia_obstaculo}

    def _obstacle_memory_decision(self) -> dict:
        if self.persistencia_obstaculo <= 0:
            self.obstaculo_clear_frames += 1
            return {"detected": False, "color": "None", "x": None,
                    "error_px": 0.0, "vision_error_norm": 0.0,
                    "mode": "no_obstacle", "priority": False, "memory_frames": 0}

        self.persistencia_obstaculo -= 1
        self.obstaculo_clear_frames  = 0

        corr = self.ultima_correccion_px * self.cfg.obstacle_fade_factor
        corr = max(-self.cfg.obstacle_max_corr_px,
                   min(self.cfg.obstacle_max_corr_px, corr))
        self.ultima_correccion_px = corr

        color = self.ultimo_color_obstaculo or "None"
        mode  = f"memory_{color.lower()}" if color != "None" else "memory_unknown"
        norm  = -(corr / max(1.0, self.cfg.correction_limit_px)) if color != "None" else 0.0

        return {"detected": False, "color": color, "x": None,
                "error_px": 0.0, "vision_error_norm": norm,
                "mode": mode, "priority": True,
                "memory_frames": self.persistencia_obstaculo}

    # ── Serial ────────────────────────────────────────────────────────────────

    def _build_serial_message(
        self,
        obs_norm:   float,
        turn_hint:  int,
        state:      str,
        prio:       int,
        mem:        int,
        pp_active:  bool,
    ) -> str:
        pp_flag = ",pp=1" if pp_active else ",pp=0"
        return (f"V2,obs={obs_norm:+.3f},turn={int(turn_hint)},"
                f"state={state},prio={prio},mem={mem}{pp_flag}")

    # ── Captura ───────────────────────────────────────────────────────────────

    def _start_capture(self):
        print(f"[CAM] cap.isOpened()={self.vision.cap.isOpened()} "
              f"threaded={self.cfg.threaded_capture}", flush=True)
        if self.cfg.threaded_capture:
            self.frame_grabber = ThreadedFrameGrabber(self.vision.cap).start()
            print("[CAM] ThreadedFrameGrabber iniciado.", flush=True)

    def _read_frame(self):
        if self.frame_grabber is not None:
            return self.frame_grabber.read()
        return self.vision.cap.read()

    def _maybe_record(self, frame: np.ndarray, fps: float):
        if not self.cfg.record_orillas:
            return
        self.record_count += 1
        if self.record_count % max(1, self.cfg.record_every_n) != 0:
            return
        if self.video_writer is None:
            out_fps = (max(self.cfg.record_fps, fps / max(1, self.cfg.record_every_n))
                       if fps > 0 else self.cfg.record_fps)
            self.video_writer = AsyncVideoWriter(
                str(self.output_file),
                frame.shape[1], frame.shape[0], out_fps,
            ).start()
            print(f"[REC] Grabando en {self.output_file}", flush=True)
        self.video_writer.write(frame.copy())

    def _write_cam_frame(self, frame: np.ndarray):
        try:
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
            with open(CAM_FRAME_PATH, "wb") as f:
                f.write(buf.tobytes())
        except Exception:
            pass

    # ── Anotación ─────────────────────────────────────────────────────────────

    def _annotate(
        self,
        frame:        np.ndarray,
        obs_info:     dict,
        turn_hint:    int,
        left_ratio:   float,
        right_ratio:  float,
        serial_msg:   str,
        fps:          float,
        pp_active:    bool,
        n_path_pts:   int,
    ):
        mode_str = f"PP({n_path_pts}pts)" if pp_active else "FALLBACK_PID"
        lines = [
            f"mode={mode_str}  fps={fps:.1f}",
            f"obs={obs_info['color']}  x={obs_info['x']}  prio={int(obs_info.get('priority',False))}",
            f"state={obs_info['mode']}  mem={obs_info.get('memory_frames',0)}",
            f"turn={turn_hint}  wL={left_ratio:.3f}  wR={right_ratio:.3f}",
            f"tx: {serial_msg[:55]}",
        ]
        y = 22
        for txt in lines:
            cv2.putText(frame, txt, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2)
            y += 22

    # ── Loop principal ────────────────────────────────────────────────────────

    def run(self, on_ready=None):
        self.serial_link.open()
        self._start_capture()

        # Warmup de cámara (esperar exposición automática estabilizada)
        print(f"[INFO] Calentando cámara ({self.cfg.warmup_frames} frames)...", flush=True)
        warmed = 0
        while warmed < self.cfg.warmup_frames:
            ret, _ = self._read_frame()
            if ret:
                warmed += 1
            else:
                time.sleep(0.01)
        print("[INFO] Cámara estabilizada.", flush=True)

        self.serial_link.send_line("READY")
        print("[INFO] READY enviado al ESP32.", flush=True)

        if on_ready is not None:
            on_ready()

        print("[INFO] Pure Pursuit runtime iniciado. ESC para detener.", flush=True)

        last_fps_time = time.perf_counter()
        fps_count     = 0
        fps           = 0.0
        lookahead_pt  = (float(C.ROBOT_BEV_X), float(C.ROBOT_BEV_Y))
        steer_deg     = 0.0

        try:
            while True:
                ret, frame = self._read_frame()
                if not ret:
                    time.sleep(0.01)
                    continue

                self.loop_count += 1
                if self.loop_count % self.cfg.process_every_n != 0:
                    time.sleep(0.001)
                    continue

                # FPS contador
                fps_count += 1
                now = time.perf_counter()
                if now - last_fps_time >= 1.0:
                    fps           = fps_count / (now - last_fps_time)
                    fps_count     = 0
                    last_fps_time = now

                # ── Visión ──────────────────────────────────────────────────
                processed_frame, positions = self.vision.process_frame(frame)

                # ── Decisión de obstáculo (siempre se calcula) ───────────────
                obs_now = self._obstacle_decision(positions, processed_frame.shape[1])
                if obs_now["detected"]:
                    obs_info = obs_now
                else:
                    self.pid_obs.prev_error = 0.0
                    obs_info = self._obstacle_memory_decision()

                # ── Detección de giro (madera en ROI superior) ───────────────
                turn_raw, left_ratio, right_ratio = self.turn_detector.detect(processed_frame)
                can_turn = (not obs_info.get("priority", False) and
                            self.obstaculo_clear_frames >= self.cfg.obstacle_clear_frames)
                turn_hint = turn_raw if can_turn else 0

                # ── Pipeline Pure Pursuit ────────────────────────────────────
                pp_active      = False
                bev_frame      = None
                path_points    = []
                bev_obstacles  = []
                obs_norm       = obs_info["vision_error_norm"]
                state          = obs_info["mode"]
                prio           = 1 if obs_info.get("priority", False) else 0
                mem            = int(obs_info.get("memory_frames", 0))

                # PP activo cuando: BEV calibrado Y sin prioridad de obstáculo activa
                if self.bev.is_calibrated and not obs_info.get("priority", False):
                    bev_frame = self.bev.warp(processed_frame)

                    # Proyectar obstáculos detectados al plano BEV
                    for color_name in ("Red", "Green"):
                        for obj in positions.get(color_name, []):
                            x, y, w, h = obj
                            result = map_obstacle_to_bev(self.bev, x, y, w, h)
                            if result is not None:
                                bev_obstacles.append((result[0], result[1], color_name))

                    # Detectar centerline en BEV
                    path_points = detect_centerline(bev_frame, bev_obstacles)

                    if len(path_points) >= C.MIN_PATH_PTS:
                        steer_deg, lookahead_pt = self.controller.compute(
                            path_points, C.ROBOT_BEV_X, C.ROBOT_BEV_Y
                        )
                        obs_norm  = self.controller.normalize(steer_deg)
                        state     = "pp_follow"
                        prio      = 0
                        mem       = 0
                        pp_active = True

                # ── Construir y enviar mensaje serial ────────────────────────
                serial_msg = self._build_serial_message(
                    obs_norm, turn_hint, state, prio, mem, pp_active
                )
                self.serial_link.send_line(serial_msg)
                serial_ack = self.serial_link.try_readline()

                log_line = f"TX: {serial_msg}"
                if serial_ack:
                    log_line += f" | RX: {serial_ack}"
                print(log_line, flush=True)

                # ── Display ──────────────────────────────────────────────────
                if self.cfg.show_window:
                    self._annotate(processed_frame, obs_info, turn_hint,
                                   left_ratio, right_ratio, serial_msg, fps,
                                   pp_active, len(path_points))

                    if bev_frame is not None:
                        bev_debug = draw_bev_debug(
                            bev_frame, path_points, lookahead_pt,
                            bev_obstacles, steer_deg, pp_active,
                        )
                        # BEV escalado al mismo alto que el frame de cámara
                        bev_h = processed_frame.shape[0]
                        bev_small = cv2.resize(bev_debug, (bev_h, bev_h))
                        combined = np.hstack([processed_frame, bev_small])
                        cv2.imshow("WRO Pure Pursuit", combined)
                    else:
                        cv2.imshow("WRO Pure Pursuit", processed_frame)

                    if cv2.waitKey(1) & 0xFF == 27:
                        break

                self._maybe_record(processed_frame, fps)
                self._write_cam_frame(processed_frame)

        finally:
            if self.frame_grabber is not None:
                self.frame_grabber.stop()
            self.vision.cap.release()
            if self.video_writer is not None:
                self.video_writer.stop()
            cv2.destroyAllWindows()
            self.serial_link.close()


# ─────────────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Pure Pursuit runtime — WRO Future Engineers"
    )
    p.add_argument("--cam-index",        type=int,   default=C.CAM_INDEX)
    p.add_argument("--serial-port",      type=str,   default=C.SERIAL_PORT)
    p.add_argument("--baudrate",         type=int,   default=C.BAUDRATE)
    p.add_argument("--process-every",    type=int,   default=C.PROCESS_EVERY)
    p.add_argument("--calib-path",       type=str,   default=None,
                   help="Ruta a bev_calib.npz (default: pure_pursuit/bev_calib.npz)")
    p.add_argument("--no-window",        action="store_true")
    p.add_argument("--threaded-capture", action="store_true", default=True)
    p.add_argument("--no-threaded-capture", action="store_true")
    p.add_argument("--record-orillas",   action="store_true")
    p.add_argument("--record-output",    type=str,   default=None)
    p.add_argument("--record-every",     type=int,   default=6)
    p.add_argument("--record-fps",       type=float, default=5.0)
    return p.parse_args()


def main():
    # ── GPIO: LED de encendido + esperar botón (igual que wro_runtime.py) ────
    _gpio = None
    try:
        import RPi.GPIO as GPIO
        _gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(27, GPIO.OUT)
        GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(27, GPIO.HIGH)
        print("[GPIO] LED encendido — Pi prendida.", flush=True)
        print("[GPIO] Esperando botón en GPIO17...", flush=True)
        while GPIO.input(17) == GPIO.LOW:
            time.sleep(0.05)
        print("[GPIO] Botón detectado. Iniciando...", flush=True)
    except ImportError:
        print("[GPIO] RPi.GPIO no disponible, omitiendo espera de botón.", flush=True)

    def led_on():
        if _gpio is not None:
            _gpio.output(27, _gpio.HIGH)
            print("[GPIO] LED encendido — sistema listo.", flush=True)

    args = parse_args()

    threaded = True
    if args.no_threaded_capture:
        threaded = False

    cfg = PPConfig(
        cam_index        = args.cam_index,
        serial_port      = args.serial_port,
        baudrate         = args.baudrate,
        process_every_n  = max(1, args.process_every),
        threaded_capture = threaded,
        show_window      = not args.no_window,
        record_orillas   = bool(args.record_orillas),
        record_output    = args.record_output,
        record_every_n   = max(1, args.record_every),
        record_fps       = max(1.0, args.record_fps),
        calib_path       = Path(args.calib_path) if args.calib_path else None,
    )

    runtime = PPRuntime(cfg)
    runtime.run(on_ready=led_on)


if __name__ == "__main__":
    main()
