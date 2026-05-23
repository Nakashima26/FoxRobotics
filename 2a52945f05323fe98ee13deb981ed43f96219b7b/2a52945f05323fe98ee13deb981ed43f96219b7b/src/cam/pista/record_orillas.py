import argparse
import time

import cv2

import detect_orillas as base


def create_writer(output_path: str, frame_width: int, frame_height: int, fps: float):
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    return cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))


def run(cam_index=1, output_path="orillas.mp4"):
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
                writer = create_writer(output_path, recorded_frame.shape[1], recorded_frame.shape[0], 30.0)

            writer.write(recorded_frame)

            cv2.imshow(base.WINDOW_NAME, recorded_frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        grabber.stop()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Detector de orillas con grabacion a video.")
    parser.add_argument("--cam", type=int, default=1, help="Indice de la camara.")
    parser.add_argument("--output", type=str, default="orillas.mp4", help="Archivo de salida del video.")
    args = parser.parse_args()
    run(cam_index=args.cam, output_path=args.output)
