#!/usr/bin/env python3
"""Draw detections onto the inspection frames and encode a video.

(a) For each inspection frame, run the real YOLOv8-seg model and draw the
    segmentation masks + boxes + class/confidence labels onto the image.
    Annotated PNGs go to web/data/annotated/NNNN.png.
(b) Encode the annotated frames into web/data/feed.mp4.

Also annotates the full-res /camera/rgb stream from the bag into the video when
--rgb-video is given (1056 frames) for a true motion feed; by default it uses
the 36 waypoint stills so it runs quickly.

Needs: ultralytics, opencv-python, numpy (all already required for live mode).
"""
import argparse
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
os.environ.setdefault("OMP_NUM_THREADS", "1")

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from pipeline import detection, weights  # noqa: E402

# BGR colours per pipeline severity for overlays.
SEV_BGR = {"severe": (92, 92, 255), "poor": (74, 178, 255),
           "fair": (102, 224, 255), "good": (141, 214, 88)}
RAW_TO_SEV = {"structural_crack": "poor", "microcrack": "fair", "spalling": "poor",
              "rebar_corrosion": "fair", "severe_distress": "severe"}


def annotate_image(cv2, np, img, result, names, conf):
    """Draw masks + boxes + labels for one ultralytics result onto img (BGR)."""
    overlay = img.copy()
    boxes = getattr(result, "boxes", None)
    masks = getattr(result, "masks", None)
    if boxes is None:
        return img
    for i, box in enumerate(boxes):
        raw = names.get(int(box.cls), str(int(box.cls)))
        c = float(box.conf)
        if c < conf:
            continue
        sev = RAW_TO_SEV.get(raw, "fair")
        color = SEV_BGR[sev]
        # mask fill
        if masks is not None and i < len(masks.data):
            m = masks.data[i].cpu().numpy().astype(bool)
            if m.shape != img.shape[:2]:
                m = cv2.resize(m.astype("uint8"), (img.shape[1], img.shape[0])) > 0
            overlay[m] = color
        # box + label
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f"{raw} {c*100:.0f}%"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
        cv2.putText(img, label, (x1 + 3, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.addWeighted(overlay, 0.4, img, 0.6, 0, img)
    return img


def _encode_h264(cv2, np, bgr_frames, out_path, fps):
    """Write an H.264 MP4 (yuv420p, browser-playable) via imageio-ffmpeg.
    Falls back to OpenCV mp4v only if imageio is unavailable."""
    frames = [cv2.cvtColor(f, cv2.COLOR_BGR2RGB) for f in bgr_frames if f is not None]
    if not frames:
        return
    h, w = frames[0].shape[:2]
    # ffmpeg (libx264) needs even dimensions.
    w2, h2 = w - (w % 2), h - (h % 2)
    try:
        import imageio.v2 as imageio
        writer = imageio.get_writer(str(out_path), fps=fps, codec="libx264",
                                    quality=8, macro_block_size=None,
                                    ffmpeg_params=["-pix_fmt", "yuv420p"])
        for f in frames:
            if (f.shape[1], f.shape[0]) != (w2, h2):
                f = cv2.resize(f, (w2, h2))
            writer.append_data(f)
        writer.close()
    except Exception as e:
        print(f"imageio H.264 failed ({e}); falling back to mp4v", flush=True)
        vw = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
        for f in bgr_frames:
            if f is not None:
                vw.write(f)
        vw.release()


def main() -> int:
    ap = argparse.ArgumentParser(description="Annotate frames + encode video")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--conf", type=float, default=0.15)
    ap.add_argument("--camera-weights", default=None)
    ap.add_argument("--fps", type=int, default=4)
    ap.add_argument("--rgb-video", action="store_true",
                    help="also annotate the full /camera/rgb stream into a motion video (slow)")
    args = ap.parse_args()

    import cv2
    import numpy as np
    from ultralytics import YOLO

    wpath = weights.resolve_camera_weights(args.camera_weights)
    model = YOLO(wpath)
    names = model.names
    print(f"model loaded: {names}", flush=True)

    out_dir = HERE / "web" / "data" / "annotated"
    out_dir.mkdir(parents=True, exist_ok=True)

    frames_dir = Path(args.bag) / "inspection_frames"
    pngs = sorted(frames_dir.glob("[0-9][0-9][0-9][0-9].png"))

    # (a) annotate each waypoint still
    annotated_paths = []
    for png in pngs:
        img = cv2.imread(str(png))
        if img is None:
            continue
        res = model.predict(str(png), conf=args.conf, verbose=False)
        for r in res:
            img = annotate_image(cv2, np, img, r, names, args.conf)
        op = out_dir / png.name
        cv2.imwrite(str(op), img)
        annotated_paths.append(op)
    print(f"annotated {len(annotated_paths)} waypoint frames -> {out_dir}", flush=True)

    # (b) encode a browser-playable H.264 video from the annotated frames.
    if annotated_paths:
        vid = HERE / "web" / "data" / "feed.mp4"
        _encode_h264(cv2, np, [cv2.imread(str(p)) for p in annotated_paths], vid, args.fps)
        print(f"wrote {vid} ({len(annotated_paths)} frames @ {args.fps} fps, H.264)", flush=True)

    # (b, optional) full-res motion video from the raw rgb stream
    if args.rgb_video:
        _rgb_video(args, cv2, np, model, names)

    return 0


def _rgb_video(args, cv2, np, model, names):
    """Decode /camera/rgb from the bag, annotate every frame, encode a video."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    from pipeline.rosbag import _find_mcap

    mcap_path = _find_mcap(args.bag)
    dec = DecoderFactory()
    img_decode = None
    frames = []
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages(topics=["/camera/rgb"]):
            if img_decode is None:
                img_decode = dec.decoder_for(channel.message_encoding, schema)
            msg = img_decode(message.data)
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, -1)
            frames.append(arr)
    if not frames:
        print("no /camera/rgb frames found", flush=True)
        return
    vid = HERE / "web" / "data" / "feed_full.mp4"
    annotated = []
    for arr in frames:
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        res = model.predict(bgr, conf=args.conf, verbose=False)
        for r in res:
            bgr = annotate_image(cv2, np, bgr, r, names, args.conf)
        annotated.append(bgr)
    _encode_h264(cv2, np, annotated, vid, 15)
    print(f"wrote {vid} ({len(frames)} rgb frames @ 15 fps, H.264)", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
