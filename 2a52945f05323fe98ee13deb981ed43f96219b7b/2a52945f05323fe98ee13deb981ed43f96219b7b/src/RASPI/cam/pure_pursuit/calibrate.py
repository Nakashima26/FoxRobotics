"""
Herramienta de calibración interactiva para BEV (Bird's Eye View).

USO:
  python -m pure_pursuit.calibrate          # desde src/RASPI/cam/
  python calibrate.py                       # desde pure_pursuit/

FLUJO:
  1. Coloca 4 marcadores en el suelo en las posiciones exactas definidas en config.py:
       A: {A_mm[0]} mm lat, {A_mm[1]} mm adelante   (izq-cerca)
       B: {B_mm[0]} mm lat, {B_mm[1]} mm adelante   (der-cerca)
       C: {C_mm[0]} mm lat, {C_mm[1]} mm adelante   (izq-lejos)
       D: {D_mm[0]} mm lat, {D_mm[1]} mm adelante   (der-lejos)
  2. Presiona 'C' para capturar el frame.
  3. Haz clic en los 4 marcadores en orden A → B → C → D.
  4. Aparece la vista BEV de preview en tiempo real.
  5. Presiona 'S' para guardar la calibración, 'R' para rehacer, ESC para salir.

La calibración se guarda en:  pure_pursuit/bev_calib.npz
"""

import os
import sys

import cv2
import numpy as np

# Agrega el directorio cam/ al path para poder importar vision.py
_HERE    = os.path.dirname(os.path.abspath(__file__))
_CAM_DIR = os.path.dirname(_HERE)
if _CAM_DIR not in sys.path:
    sys.path.insert(0, _CAM_DIR)

from vision import open_camera
from .bev import BEVTransformer
from . import config as C

# Etiquetas y colores de los 4 puntos de calibración
POINT_LABELS = ["A (izq-cerca)", "B (der-cerca)", "C (izq-lejos)", "D (der-lejos)"]
POINT_COLORS = [
    (0, 255, 255),   # amarillo-cyan
    (0, 200, 255),
    (0, 140, 255),
    (0,  80, 255),
]


def _draw_instructions(frame: np.ndarray, clicks: list, done: bool) -> np.ndarray:
    out = frame.copy()
    h, w = out.shape[:2]

    # Puntos ya clickeados
    for i, (px, py) in enumerate(clicks):
        cv2.circle(out, (px, py), 8, POINT_COLORS[i], -1)
        cv2.circle(out, (px, py), 8, (0, 0, 0), 2)
        cv2.putText(out, POINT_LABELS[i], (px + 10, py - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, POINT_COLORS[i], 2)

    # Siguiente punto esperado
    if not done:
        idx = len(clicks)
        msg = f"Clic en punto {POINT_LABELS[idx]}"
        cv2.putText(out, msg, (10, h - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2)
    else:
        cv2.putText(out, "S = guardar   R = rehacer   ESC = salir",
                    (10, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    return out


def _draw_bev_preview(bev_img: np.ndarray, dst_pts: np.ndarray) -> np.ndarray:
    out = bev_img.copy()
    robot_x, robot_y = C.ROBOT_BEV_X, C.ROBOT_BEV_Y

    # Puntos destino esperados
    for i, (dx, dy) in enumerate(dst_pts):
        cv2.circle(out, (int(dx), int(dy)), 7, POINT_COLORS[i], -1)
        cv2.putText(out, POINT_LABELS[i][0], (int(dx) + 4, int(dy) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, POINT_COLORS[i], 2)

    # Robot
    cv2.circle(out, (robot_x, robot_y), 9, (255, 80, 0), -1)
    cv2.arrowedLine(out, (robot_x, robot_y), (robot_x, robot_y - 30),
                    (255, 255, 255), 2, tipLength=0.3)

    cv2.putText(out, "BEV preview", (6, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
    return out


def run_calibration(cam_index: int = C.CAM_INDEX) -> None:
    cap = open_camera(cam_index)

    # ── Paso 1: capturar frame ─────────────────────────────────────────────────
    print("Encuadra la cámara sobre el suelo y presiona 'C' para capturar.", flush=True)
    captured_frame = None
    cv2.namedWindow("Calibracion BEV — Captura", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibracion BEV — Captura", 1280, 720)
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        frame = cv2.flip(frame, 1)   # mismo flip que vision.py
        cv2.putText(frame, "Presiona C para capturar | ESC para salir",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.imshow("Calibracion BEV — Captura", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') or key == ord('C'):
            captured_frame = frame.copy()
            break
        if key == 27:
            cap.release()
            cv2.destroyAllWindows()
            return

    cv2.destroyWindow("Calibracion BEV — Captura")

    # ── Paso 2: clic en 4 puntos ──────────────────────────────────────────────
    clicks: list[tuple[int, int]] = []
    bev = BEVTransformer()
    dst_pts = BEVTransformer.expected_dst_pts()

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))

    cv2.namedWindow("Calibracion BEV — Puntos", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibracion BEV — Puntos", 1280, 720)

    cv2.namedWindow("Calibracion BEV — Preview BEV", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibracion BEV — Preview BEV", 1280, 720)

    cv2.setMouseCallback("Calibracion BEV — Puntos", on_mouse)

    print("\nHaz clic en los 4 marcadores en orden: A → B → C → D", flush=True)
    for lbl in POINT_LABELS:
        mm_idx = POINT_LABELS.index(lbl)
        mm = C.CALIB_REAL_MM[mm_idx]
        print(f"  {lbl}: {mm[0]:+.0f} mm lateral, {mm[1]:.0f} mm adelante", flush=True)

    live_cap_running = True

    while True:
        done = len(clicks) == 4
        display = _draw_instructions(captured_frame, clicks, done)

        # Preview BEV en tiempo real al tener los 4 puntos
        if done:
            src_pts = np.float32(clicks)
            try:
                bev.save(src_pts)
            except RuntimeError as e:
                print(e, flush=True)
                clicks.clear()
                continue

            ret, live_frame = cap.read()
            if ret:
                live_frame = cv2.flip(live_frame, 1)
                bev_img = bev.warp(live_frame)
                if bev_img is not None:
                    preview = _draw_bev_preview(bev_img, dst_pts)
                    cv2.imshow("Calibracion BEV — Preview BEV", preview)

        cv2.imshow("Calibracion BEV — Puntos", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s') or key == ord('S'):
            if done:
                print(f"\n[OK] Calibración guardada en {C.CALIB_FILE}", flush=True)
                break
            else:
                print("Aún faltan puntos por seleccionar.", flush=True)

        elif key == ord('r') or key == ord('R'):
            clicks.clear()
            bev.H = None
            bev.H_inv = None
            print("Rehacer — haz clic en los 4 puntos de nuevo.", flush=True)

        elif key == 27:
            print("Calibración cancelada.", flush=True)
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Calibración BEV para Pure Pursuit WRO")
    parser.add_argument("--cam-index", type=int, default=C.CAM_INDEX)
    args = parser.parse_args()
    run_calibration(args.cam_index)


if __name__ == "__main__":
    main()
