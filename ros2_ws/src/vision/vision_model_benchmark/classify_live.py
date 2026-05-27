from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a YOLO classification (.pt) model on a camera or image and print top-1 label.",
    )
    parser.add_argument(
        "--model",
        required=True,
        help="Path to the classification .pt file (e.g. gender_cls.pt).",
    )
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument(
        "--source",
        default="/dev/video10",
        help="Camera device or index for live mode.",
    )
    source_group.add_argument(
        "--image",
        help="Path to a single image. If set, runs once and exits.",
    )
    parser.add_argument("--width", type=int, default=640, help="Requested camera width.")
    parser.add_argument("--height", type=int, default=480, help="Requested camera height.")
    parser.add_argument("--imgsz", type=int, default=224, help="Classifier input size (YOLO-cls default is 224).")
    parser.add_argument(
        "--print-interval",
        type=float,
        default=0.5,
        help="Minimum seconds between live prints.",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=2,
        help="How many top-k classes to print each tick.",
    )
    parser.add_argument(
        "--save-latest",
        default=None,
        help="If set, write the most recent annotated frame (camera image + label overlay) to this path each print tick.",
    )
    return parser.parse_args()


def open_camera(source: str, width: int, height: int) -> cv2.VideoCapture:
    camera_source = int(source) if source.isdigit() else source
    cap = cv2.VideoCapture(camera_source, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera source: {source}")
    return cap


def format_topk(result, top: int) -> str:
    probs = result.probs
    if probs is None:
        return "no probs in result (is this a classification model?)"
    names = result.names
    indices = probs.top5[:top] if hasattr(probs, "top5") else [int(probs.top1)]
    parts = []
    for idx in indices:
        idx = int(idx)
        label = str(names[idx]) if isinstance(names, dict) else str(names[idx])
        conf = float(probs.data[idx]) if hasattr(probs, "data") else float(probs.top1conf)
        parts.append(f"{label} conf={conf:.2f}")
    return " | ".join(parts)


def run_image(model: YOLO, image_path: str, imgsz: int, top: int) -> None:
    path = Path(image_path)
    if not path.is_file():
        raise FileNotFoundError(f"Image not found: {image_path}")
    result = model.predict(str(path), imgsz=imgsz, verbose=False)[0]
    print(f"{path.name}: {format_topk(result, top)}")


def run_live(
    model: YOLO,
    source: str,
    width: int,
    height: int,
    imgsz: int,
    top: int,
    print_interval: float,
    save_latest: str | None,
) -> None:
    cap = open_camera(source, width, height)
    print(f"Live classify started. source={source} imgsz={imgsz}. Press Ctrl+C to stop.")
    if save_latest:
        print(f"Saving annotated frames to {save_latest}")
    last_print = 0.0
    frame_count = 0
    start = time.monotonic()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("camera read failed")
                time.sleep(0.2)
                continue
            result = model.predict(frame, imgsz=imgsz, verbose=False)[0]
            frame_count += 1
            now = time.monotonic()
            if now - last_print >= print_interval:
                last_print = now
                elapsed = now - start
                fps = frame_count / elapsed if elapsed > 0.0 else 0.0
                label_text = format_topk(result, top)
                print(f"{elapsed:7.2f}s fps={fps:4.2f} | {label_text}")
                if save_latest:
                    annotated = frame.copy()
                    cv2.putText(
                        annotated,
                        label_text,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imwrite(save_latest, annotated)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cap.release()


def main() -> None:
    args = parse_args()
    model = YOLO(args.model)
    if args.image:
        run_image(model, args.image, args.imgsz, args.top)
    else:
        run_live(
            model,
            args.source,
            args.width,
            args.height,
            args.imgsz,
            args.top,
            args.print_interval,
            args.save_latest,
        )


if __name__ == "__main__":
    main()
