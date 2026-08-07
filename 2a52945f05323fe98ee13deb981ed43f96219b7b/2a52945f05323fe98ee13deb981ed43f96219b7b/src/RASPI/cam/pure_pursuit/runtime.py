"""
Runtime Pure Pursuit — WRO Future Engineers.

Punto de entrada principal del módulo.  Es el pipeline de test_vision.py
(cámara → BEV → centerline → Pure Pursuit) más el envío serial al ESP32.

El carrito SIEMPRE intenta seguir la línea central.  Los obstáculos rojo/verde
ya están metidos dentro de la línea: detect_centerline los infla con el sesgo de
color WRO, así que la centerline pasa por el lado correcto y el robot simplemente
la sigue.  No hay un sistema de esquive separado.

Pipeline por frame:
  1. Captura (ThreadedFrameGrabber, mismo que wro_runtime.py)
  2. vision.process_frame() → detección HSV de obstáculos rojo/verde
  3. bev.warp() → map_obstacle_to_bev() → detect_centerline()
  4. PurePursuitController.compute() → steer_deg → obs = steer_deg/35
  5. SerialLink.send_line() → ESP32  (mensaje V2 con pp=1)

Protocolo serial V2:
  V2,obs=+0.350,turn=0,state=pp_follow,prio=0,mem=0,pp=1   (siguiendo línea)
  V2,obs=+0.000,turn=0,state=no_path,prio=0,mem=0,pp=1     (sin línea → recto)

  • pp=1 → ESP32 usa ppSteerGain (35°) para recuperar steer_deg.

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


# ─────────────────────────────────────────────────────────────────────────────

class PPRuntime:
    """
    Runtime Pure Pursuit puro: sigue la centerline y manda steer_deg al ESP32.
    La infraestructura de captura, serial y GPIO es idéntica a wro_runtime.py.
    """

    def __init__(self, cfg: PPConfig):
        self._last_bev_obstacles = []

        self.cfg = cfg

        # Visión
        self.vision     = Vision(cfg.cam_index)
        self.bev        = BEVTransformer(cfg.calib_path)
        self.controller = PurePursuitController()

        # Serial
        self.serial_link = SerialLink(cfg.serial_port, cfg.baudrate)

        # Captura / grabación
        self.loop_count    = 0
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
            print("[PP] Sin calibración BEV — el robot irá recto.", flush=True)
            print(f"[PP] Corre: python -m pure_pursuit.calibrate", flush=True)

    # ── Serial ────────────────────────────────────────────────────────────────

    def _build_serial_message(self, obs_norm: float, state: str) -> str:
        # prio=1 solo si REALMENTE hay un obstáculo detectado cerca, no por steer alto
        has_obstacle = len(self._last_bev_obstacles) > 0  # ver nota abajo
        return (f"V2,obs={obs_norm:+.3f},turn=0,"
                f"state={state},prio={int(has_obstacle)},mem=0,pp=1")

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
        steer_deg:    float,
        obs_norm:     float,
        pp_active:    bool,
        n_path_pts:   int,
        positions:    dict,
        serial_msg:   str,
        fps:          float,
    ):
        lines = [
            f"fps={fps:.1f}  pp={'ON' if pp_active else 'OFF'}  pts={n_path_pts}",
            f"steer={steer_deg:+.1f} deg  obs={obs_norm:+.3f}",
            f"obs_R={len(positions.get('Red', []))}  obs_G={len(positions.get('Green', []))}",
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
                frame = cv2.flip(frame, 1)
                processed_frame, positions = self.vision.process_frame(frame)

                # ── Pipeline Pure Pursuit (idéntico a test_vision.py) ────────
                steer_deg     = 0.0
                obs_norm      = 0.0
                lookahead_pt  = (float(C.ROBOT_BEV_X), float(C.ROBOT_BEV_Y))
                path_points   = []
                bev_frame     = None
                bev_obstacles = []
                pp_active     = False

                if self.bev.is_calibrated:
                    try:
                        bev_frame = self.bev.warp(processed_frame)

                        # Proyectar obstáculos detectados al plano BEV
                        for color_name in ("Red", "Green"):
                            for obj in positions.get(color_name, []):
                                x, y, w, h = obj
                                result = map_obstacle_to_bev(self.bev, x, y, w, h)
                                if result is not None:
                                    bev_obstacles.append((result[0], result[1], color_name))

                        # Detectar centerline (con obstáculos ya inflados)
                        path_points = detect_centerline(bev_frame, bev_obstacles)

                        if len(path_points) >= C.MIN_PATH_PTS:
                            steer_deg, lookahead_pt = self.controller.compute(
                                path_points, C.ROBOT_BEV_X, C.ROBOT_BEV_Y
                            )
                            obs_norm  = self.controller.normalize(steer_deg)
                            pp_active = True
                    except Exception as e:
                        import traceback
                        print(f"[ERROR] {e}", flush=True)
                        traceback.print_exc()

                # Sin línea válida → recto (obs=0).  En teoría siempre hay línea.
                state = "pp_follow" if pp_active else "no_path"

                bev_obstacles = []
                for color_name in ("Red", "Green"):
                    for obj in positions.get(color_name, []):
                        ...
                        bev_obstacles.append(...)

                # ← agrega esta línea aquí
                self._last_bev_obstacles = bev_obstacles

                path_points = detect_centerline(bev_frame, bev_obstacles)

                # ── Construir y enviar mensaje serial ────────────────────────
                serial_msg = self._build_serial_message(obs_norm, state)
                self.serial_link.send_line(serial_msg)
                serial_ack = self.serial_link.try_readline()

                log_line = f"TX: {serial_msg}"
                if serial_ack:
                    log_line += f" | RX: {serial_ack}"
                print(log_line, flush=True)

                # ── Display ──────────────────────────────────────────────────
                if self.cfg.show_window:
                    self._annotate(processed_frame, steer_deg, obs_norm,
                                   pp_active, len(path_points), positions,
                                   serial_msg, fps)

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
