import argparse
import re
import threading
import time
from queue import Empty, Full, Queue
from pathlib import Path
from typing import Optional

import cv2

import detect_orillas as base


OUTPUT_PATTERN = re.compile(r"^orillas(\d+)\.mp4$")


def create_writer(output_path: str, frame_width: int, frame_height: int, fps: float):
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    return cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))


class AsyncVideoWriter:
    def __init__(self, output_path: str, frame_width: int, frame_height: int, fps: float, max_queue_size: int = 8):
        self.writer = create_writer(output_path, frame_width, frame_height, fps)
        self.queue = Queue(maxsize=max_queue_size)
        self.stopped = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        while not self.stopped or not self.queue.empty():
            try:
                frame = self.queue.get(timeout=0.05)
            except Empty:
                continue
            try:
                self.writer.write(frame)
            finally:
                self.queue.task_done()

    def write(self, frame):
        if self.stopped:
            return
        try:
            self.queue.put_nowait(frame)
        except Full:
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except Empty:
                pass
            try:
                self.queue.put_nowait(frame)
            except Full:
                pass

    def stop(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self.writer.release()


def next_output_path(directory: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)

    highest_index = 0
    for candidate in directory.glob("orillas*.mp4"):
        match = OUTPUT_PATTERN.match(candidate.name)
        if match:
            highest_index = max(highest_index, int(match.group(1)))

    return directory / f"orillas{highest_index + 1}.mp4"


def resolve_output_path(output_path: Optional[str]) -> Path:
    if output_path:
        path = Path(output_path)
        if path.suffix.lower() == ".mp4" and OUTPUT_PATTERN.match(path.name):
            return next_output_path(path.parent)
        if path.is_dir() or not path.suffix:
            return next_output_path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    return next_output_path(Path.cwd())


def run(cam_index=1, output_path=None):
    output_file = resolve_output_path(output_path)
    grabber = base.FrameGrabber(cam_index).start()
    previous_left = None
    previous_right = None
    last_fps_time = time.perf_counter()
    fps = 0.0
    frame_count = 0
    loop_index = 0
    last_debug = None
    writer = None

    cv2.namedWindow(base.WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(base.WINDOW_NAME, base.WINDOW_WIDTH, base.WINDOW_HEIGHT)

    try:
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

            analysis = cv2.resize(frame, (base.ANALYSIS_WIDTH, base.ANALYSIS_HEIGHT), interpolation=cv2.INTER_AREA)

            if loop_index % base.PROCESS_EVERY_N == 0 or last_debug is None:
                last_debug = base.detect_turn_direction(analysis, previous_left, previous_right)
                previous_left = last_debug["left_ratio"]
                previous_right = last_debug["right_ratio"]

            debug = last_debug
            loop_index += 1

            recorded_frame = analysis.copy()
            base.draw_debug(recorded_frame, debug, fps=fps)

            if writer is None:
                writer = AsyncVideoWriter(
                    str(output_file),
                    recorded_frame.shape[1],
                    recorded_frame.shape[0],
                    30.0,
                ).start()

            writer.write(recorded_frame.copy())

            cv2.imshow(base.WINDOW_NAME, recorded_frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        grabber.stop()
        if writer is not None:
            writer.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Detector de orillas con grabacion a video.")
    parser.add_argument("--cam", type=int, default=1, help="Indice de la camara.")
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Archivo o carpeta de salida. Si no se indica, crea orillasN.mp4 en la carpeta actual.",
    )
    args = parser.parse_args()
    run(cam_index=args.cam, output_path=args.output)
