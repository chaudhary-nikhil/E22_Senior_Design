#!/usr/bin/env python3
"""
Extract 2D hand trajectory from lab swim-stroke video using MediaPipe Hands.

Used to compare against IMU-integrated stroke paths when improving visualization.

Dependencies (install on the machine that runs wifi_session_processor.py):
    pip install -r requirements-calibration.txt

Output JSON uses image-normalized coordinates (0–1, origin top-left) plus pixel
coordinates for the chosen landmark (default: wrist).
"""
from __future__ import annotations

import json
import math
import os
import warnings
from typing import Any, Dict, List, Optional, Tuple

# Before MediaPipe / TensorFlow Lite native code loads: cut INFO spam on stderr.
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
os.environ.setdefault("GLOG_minloglevel", "2")
warnings.filterwarnings(
    "ignore",
    category=UserWarning,
    module=r"google\.protobuf\.symbol_database",
)


def _require_cv():
    try:
        import cv2  # noqa: F401
        return cv2
    except ImportError as e:
        raise RuntimeError(
            "OpenCV is required. From the dashboard folder run: "
            "python3 -m pip install -r requirements-calibration.txt"
        ) from e


def _require_mp():
    try:
        import mediapipe as mp  # noqa: F401
        try:
            from absl import logging as absl_logging
            absl_logging.set_verbosity(absl_logging.ERROR)
        except Exception:
            pass
        return mp
    except ImportError as e:
        raise RuntimeError(
            "MediaPipe is required. From the dashboard folder run: "
            "python3 -m pip install -r requirements-calibration.txt"
        ) from e


def _moving_average_xy(
    xs: List[Optional[float]],
    ys: List[Optional[float]],
    window: int,
) -> Tuple[List[Optional[float]], List[Optional[float]]]:
    if window < 2:
        return xs, ys
    half = window // 2
    n = len(xs)
    ox: List[Optional[float]] = [None] * n
    oy: List[Optional[float]] = [None] * n
    for i in range(n):
        acc_x, acc_y, c = 0.0, 0.0, 0
        for j in range(max(0, i - half), min(n, i + half + 1)):
            if xs[j] is not None and ys[j] is not None:
                acc_x += xs[j]  # type: ignore
                acc_y += ys[j]  # type: ignore
                c += 1
        if c > 0:
            ox[i] = acc_x / c
            oy[i] = acc_y / c
    return ox, oy


def _resolve_hand_preference(
    preference: str,
    left_score: int,
    right_score: int,
) -> str:
    pref = (preference or "auto").strip().lower()
    if pref in ("left", "l"):
        return "Left"
    if pref in ("right", "r"):
        return "Right"
    if left_score >= right_score:
        return "Left"
    return "Right"


def process_video_bytes(
    video_bytes: bytes,
    *,
    hand_preference: str = "auto",
    frame_stride: int = 1,
    smooth_window: int = 0,
    landmark_index: int = 0,
) -> Dict[str, Any]:
    """
    Run MediaPipe Hands on every `frame_stride` frame of an in-memory video file.

    :param landmark_index: MediaPipe hand landmark index (0 = WRIST).
    """
    cv2 = _require_cv()
    mp = _require_mp()

    import tempfile

    with tempfile.NamedTemporaryFile(suffix=".mp4", delete=False) as tmp:
        tmp.write(video_bytes)
        path = tmp.name
    try:
        return process_video_path(
            path,
            hand_preference=hand_preference,
            frame_stride=frame_stride,
            smooth_window=smooth_window,
            landmark_index=landmark_index,
            _cv2=cv2,
            _mp=mp,
        )
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def process_video_path(
    path: str,
    *,
    hand_preference: str = "auto",
    frame_stride: int = 1,
    smooth_window: int = 0,
    landmark_index: int = 0,
    _cv2=None,
    _mp=None,
) -> Dict[str, Any]:
    cv2 = _cv2 or _require_cv()
    mp = _mp or _require_mp()

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return {
            "ok": False,
            "error": "Could not open video file (unsupported or corrupt).",
        }

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
    frame_count_est = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)

    if fps <= 1e-6:
        fps = 30.0

    mp_hands = mp.solutions.hands
    # landmark_index 0 = wrist; matches stroke visualization focus
    left_score = 0
    right_score = 0

    per_frame_hands: List[Tuple[int, List[Dict[str, Any]]]] = []
    frame_index = 0
    processed = 0
    stride = max(1, int(frame_stride))

    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    ) as hands:
        while True:
            ok, image = cap.read()
            if not ok:
                break
            if frame_index % stride != 0:
                frame_index += 1
                continue

            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image_rgb.flags.writeable = False
            results = hands.process(image_rgb)

            hands_row: List[Dict[str, Any]] = []
            if results.multi_hand_landmarks and results.multi_handedness:
                for lm_set, handed in zip(
                    results.multi_hand_landmarks,
                    results.multi_handedness,
                ):
                    label = handed.classification[0].label
                    idx = min(landmark_index, len(lm_set.landmark) - 1)
                    pt = lm_set.landmark[idx]
                    conf = float(handed.classification[0].score)
                    hands_row.append({
                        "label": label,
                        "x_norm": float(pt.x),
                        "y_norm": float(pt.y),
                        "z_norm": float(pt.z),
                        "confidence": conf,
                    })
                    if label == "Left":
                        left_score += 1
                    elif label == "Right":
                        right_score += 1

            per_frame_hands.append((frame_index, hands_row))
            processed += 1
            frame_index += 1

    cap.release()

    chosen = _resolve_hand_preference(hand_preference, left_score, right_score)

    xs: List[Optional[float]] = []
    ys: List[Optional[float]] = []
    zs: List[Optional[float]] = []
    detected_flags: List[bool] = []

    for _vf, hands_row in per_frame_hands:
        match = [h for h in hands_row if h.get("label") == chosen]
        if not match:
            # If auto-selected hand missing but the other is present: mark not detected
            xs.append(None)
            ys.append(None)
            zs.append(None)
            detected_flags.append(False)
            continue
        # If two hands same label (impossible) take highest confidence
        best = max(match, key=lambda h: h.get("confidence", 0.0))
        xs.append(best["x_norm"])
        ys.append(best["y_norm"])
        zs.append(best.get("z_norm"))
        detected_flags.append(True)

    if smooth_window >= 2:
        xs, ys = _moving_average_xy(xs, ys, smooth_window)
        # Re-mark detection where smoothed values exist
        detected_flags = [x is not None and y is not None for x, y in zip(xs, ys)]

    samples: List[Dict[str, Any]] = []
    frames_with_target = 0
    frames_with_any = 0
    for _vf, hands_row in per_frame_hands:
        if hands_row:
            frames_with_any += 1
    for i, (video_frame, _hands_row) in enumerate(per_frame_hands):
        t_sec = video_frame / fps
        xn, yn = xs[i], ys[i]
        zn = zs[i] if i < len(zs) else None
        det = detected_flags[i] if i < len(detected_flags) else False
        if det:
            frames_with_target += 1
        x_px = (xn * width) if (det and xn is not None) else None
        y_px = (yn * height) if (det and yn is not None) else None
        samples.append({
            "frame": int(video_frame),
            "sample_index": i,
            "t_sec": round(t_sec, 6),
            "x_norm": None if xn is None else round(float(xn), 6),
            "y_norm": None if yn is None else round(float(yn), 6),
            "z_norm": None if zn is None else round(float(zn), 6),
            "x_px": None if x_px is None else int(round(x_px)),
            "y_px": None if y_px is None else int(round(y_px)),
            "detected": det,
        })

    duration = processed / fps if fps > 0 else 0.0

    # Path length in normalized space (for quick QC)
    path_len = 0.0
    prev: Optional[Tuple[float, float]] = None
    for s in samples:
        if not s["detected"] or s["x_norm"] is None or s["y_norm"] is None:
            continue
        xy = (float(s["x_norm"]), float(s["y_norm"]))
        if prev is not None:
            path_len += math.hypot(xy[0] - prev[0], xy[1] - prev[1])
        prev = xy

    return {
        "ok": True,
        "tool": "goldenform_stroke_video_track",
        "schema_version": 1,
        "video": {
            "width": width,
            "height": height,
            "fps": round(fps, 4),
            "frame_count_reported": frame_count_est,
            "frames_processed": processed,
            "frame_stride": max(1, frame_stride),
            "duration_sec": round(duration, 4),
        },
        "tracking": {
            "landmark_index": landmark_index,
            "landmark_name": "WRIST" if landmark_index == 0 else f"landmark_{landmark_index}",
            "hand_chosen": chosen,
            "hand_preference": hand_preference,
            "left_detections": left_score,
            "right_detections": right_score,
            "smooth_window": smooth_window,
        },
        "stats": {
            "frames_with_chosen_hand": frames_with_target,
            "frames_with_any_hand": frames_with_any,
            "frames_total": processed,
            "path_length_norm": round(path_len, 6),
        },
        "alignment_hint": (
            "Sync with IMU: align t_sec with your session JSON timestamps (s or ms). "
            "Record a sharp motion or clap visible in both video and IMU window."
        ),
        "samples": samples,
    }


def main_cli():
    """Optional: python -m stroke_video_track /path/to/video.mp4 > out.json"""
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 stroke_video_track.py <video.mp4>", file=sys.stderr)
        sys.exit(1)
    out = process_video_path(sys.argv[1])
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main_cli()
