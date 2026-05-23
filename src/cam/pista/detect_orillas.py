import os
import time
import threading

import cv2
import numpy as np


# Rango HSV para madera/beige/cafe claro.
# Ajustable segun la luz real de la pista.
WOOD_LOWER = np.array([5, 25, 40])
WOOD_UPPER = np.array([35, 180, 255])
ANALYSIS_WIDTH = 1640
ANALYSIS_HEIGHT = 1232
PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 480
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480
PROCESS_EVERY_N = 3
WINDOW_NAME = "Pista"


cv2.setUseOptimized(True)
cpu_count = os.cpu_count() or 1
cv2.setNumThreads(min(4, cpu_count))


def open_camera(cam_index: int):
    if os.name == "nt":
        cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(cam_index)
    else:
        pipeline = (
            "libcamerasrc ! queue max-size-buffers=1 leaky=downstream "
            "! video/x-raw, width=1640, height=1232, framerate=30/1 "
            "! videoconvert ! videoscale ! video/x-raw, width=320, height=240 "
            "! appsink drop=true max-buffers=1 sync=false"
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            cap = cv2.VideoCapture(cam_index)

    if not cap.isOpened():
        raise RuntimeError("No se pudo abrir la camara.")

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class FrameGrabber:
    def __init__(self, cam_index: int):
        self.cap = open_camera(cam_index)
        self.lock = threading.Lock()
        self.frame = None
        self.stopped = False
        self.thread = threading.Thread(target=self._update, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                self.stopped = True
                break
            with self.lock:
                self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return True, self.frame.copy()

    def stop(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.cap.release()


def detect_turn_direction(frame, previous_left=None, previous_right=None, top_ratio=0.30):
    height, width = frame.shape[:2]
    top_limit = max(1, int(height * top_ratio))
    roi = frame[:top_limit, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, WOOD_LOWER, WOOD_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    mid = width // 2
    left_mask = mask[:, :mid]
    right_mask = mask[:, mid:]

    left_count = int(cv2.countNonZero(left_mask))
    right_count = int(cv2.countNonZero(right_mask))

    left_ratio = left_count / max(1, left_mask.size)
    right_ratio = right_count / max(1, right_mask.size)

    left_drop = previous_left is not None and left_ratio < previous_left * 0.85
    right_drop = previous_right is not None and right_ratio < previous_right * 0.85

    turn_text = None
    if left_drop and not right_drop:
        turn_text = "girar a la izquierda"
        print(turn_text, flush=True)
    elif right_drop and not left_drop:
        turn_text = "girar a la derecha"
        print(turn_text, flush=True)

    debug = {
        "mask": mask,
        "left_ratio": left_ratio,
        "right_ratio": right_ratio,
        "top_ratio": top_ratio,
        "turn_text": turn_text,
    }
    return debug


def draw_debug(frame, debug):
    height, width = frame.shape[:2]
    top_limit = max(1, int(height * debug["top_ratio"]))
    mid = width // 2

    cv2.rectangle(frame, (0, 0), (mid - 1, top_limit - 1), (0, 255, 0), 1)
    cv2.rectangle(frame, (mid, 0), (width - 1, top_limit - 1), (0, 255, 255), 1)

    overlay = frame.copy()
    resized_mask = cv2.resize(debug["mask"], (width, height), interpolation=cv2.INTER_NEAREST)
    colored_mask = cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2BGR)
    overlay[:top_limit, :] = cv2.addWeighted(
        frame[:top_limit, :],
        0.65,
        colored_mask[:top_limit, :],
        0.35,
        0,
    )
    frame[:top_limit, :] = overlay[:top_limit, :]

    cv2.putText(
        frame,
        f"L={debug['left_ratio']:.3f} R={debug['right_ratio']:.3f}",
        (10, top_limit + 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    if debug["turn_text"]:
        cv2.putText(
            frame,
            debug["turn_text"],
            (10, top_limit + 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )


def run(cam_index=1):
    grabber = FrameGrabber(cam_index).start()
    previous_left = None
    previous_right = None
    last_fps_time = time.perf_counter()
    fps = 0.0
    frame_count = 0
    loop_index = 0
    last_debug = None

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

    while True:
        ret, frame = grabber.read()
        if not ret:
            time.sleep(0.005)
            continue

        frame_count += 1
        now = time.perf_counter()
        elapsed = now - last_fps_time
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            last_fps_time = now

        analysis = cv2.resize(frame, (ANALYSIS_WIDTH, ANALYSIS_HEIGHT), interpolation=cv2.INTER_AREA)
        preview = cv2.resize(analysis, (PREVIEW_WIDTH, PREVIEW_HEIGHT), interpolation=cv2.INTER_AREA)

        if loop_index % PROCESS_EVERY_N == 0 or last_debug is None:
            last_debug = detect_turn_direction(analysis, previous_left, previous_right)
            previous_left = last_debug["left_ratio"]
            previous_right = last_debug["right_ratio"]

        debug = last_debug
        loop_index += 1

        draw_debug(preview, debug)
        cv2.putText(
            preview,
            f"FPS: {fps:.1f}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        cv2.imshow(WINDOW_NAME, preview)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    grabber.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
