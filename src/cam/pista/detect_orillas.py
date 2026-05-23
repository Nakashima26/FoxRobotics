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
WINDOW_WIDTH = 320
WINDOW_HEIGHT = 240
PROCESS_EVERY_N = 3
WINDOW_NAME = "Pista"
TOP_FIFTHS = 5
EDGE_FIFTH_INDEXES = (0, 4)


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

    fifth_width = max(1, width // TOP_FIFTHS)
    left_mask = mask[:, :fifth_width]
    right_mask = mask[:, fifth_width * (TOP_FIFTHS - 1):]

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
        "fifth_width": fifth_width,
        "turn_text": turn_text,
    }
    return debug


def draw_debug(frame, debug, font_scale=0.6, thickness=2):
    height, width = frame.shape[:2]
    top_limit = max(1, int(height * debug["top_ratio"]))
    fifth_width = max(1, debug["fifth_width"])

    for index in range(1, TOP_FIFTHS):
        x = min(width - 1, index * fifth_width)
        cv2.line(frame, (x, 0), (x, top_limit - 1), (255, 255, 0), 1)

    cv2.rectangle(frame, (0, 0), (fifth_width - 1, top_limit - 1), (0, 255, 0), 1)
    cv2.rectangle(frame, (width - fifth_width, 0), (width - 1, top_limit - 1), (0, 255, 255), 1)

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
        font_scale,
        (255, 255, 255),
        thickness,
    )
    if debug["turn_text"]:
        cv2.putText(
            frame,
            debug["turn_text"],
            (10, top_limit + 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 255),
            thickness,
        )


def get_window_scale(default_width=PREVIEW_WIDTH, default_height=PREVIEW_HEIGHT):
    try:
        _, _, win_width, win_height = cv2.getWindowImageRect(WINDOW_NAME)
        if win_width <= 0 or win_height <= 0:
            return 1.0
        return min(win_width / default_width, win_height / default_height)
    except cv2.error:
        return 1.0


def get_window_size(default_width=PREVIEW_WIDTH, default_height=PREVIEW_HEIGHT):
    try:
        _, _, win_width, win_height = cv2.getWindowImageRect(WINDOW_NAME)
        if win_width <= 0 or win_height <= 0:
            return default_width, default_height
        return win_width, win_height
    except cv2.error:
        return default_width, default_height


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
        window_width, window_height = get_window_size()
        display = cv2.resize(analysis, (window_width, window_height), interpolation=cv2.INTER_AREA)
        window_scale = min(window_width / PREVIEW_WIDTH, window_height / PREVIEW_HEIGHT)
        font_scale = max(0.4, 0.6 * window_scale)
        thickness = max(1, int(round(2 * window_scale)))

        if loop_index % PROCESS_EVERY_N == 0 or last_debug is None:
            last_debug = detect_turn_direction(analysis, previous_left, previous_right)
            previous_left = last_debug["left_ratio"]
            previous_right = last_debug["right_ratio"]

        debug = last_debug
        loop_index += 1

        draw_debug(display, debug, font_scale=font_scale, thickness=thickness)
        cv2.putText(
            display,
            f"FPS: {fps:.1f}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 255, 0),
            thickness,
        )
        cv2.imshow(WINDOW_NAME, display)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    grabber.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
