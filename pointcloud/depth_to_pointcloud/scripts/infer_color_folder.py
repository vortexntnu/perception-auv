#!/usr/bin/env python3
"""Run YOLO-OBB inference on every color image in a folder.

For each input image `<input_dir>/frame_XXXXXX.png`:
  - draw oriented bounding boxes onto a copy
  - write the annotated image to `<output_dir>/annotated/frame_XXXXXX.png`
  - write detections as JSON to `<output_dir>/detections/frame_XXXXXX.json`
"""

import argparse
import json
import os
from glob import glob

import cv2
import numpy as np
from ultralytics import YOLO


def draw_obb(img, cx, cy, w, h, theta, color=(0, 255, 0), thickness=2):
    rect = ((float(cx), float(cy)), (float(w), float(h)), float(np.degrees(theta)))
    box = cv2.boxPoints(rect)
    box = np.intp(box)
    cv2.polylines(img, [box], True, color, thickness)
    return box


def process_frame(frame, model, conf, device, imgsz):
    results = model.predict(
        frame, conf=conf, device=device, imgsz=imgsz, verbose=False)

    detections = []
    annotated = frame.copy()

    for r in results:
        if r.obb is None:
            raise RuntimeError(
                "Loaded model does not output OBB predictions. "
                "Make sure you are using a YOLO-OBB model.")

        xywhr = r.obb.xywhr.cpu().numpy()
        confs = r.obb.conf.cpu().numpy()
        clss = r.obb.cls.cpu().numpy()

        for (cx, cy, w, h, theta), score, cls_id in zip(xywhr, confs, clss):
            cls_id = int(cls_id)
            score = float(score)
            theta = float(theta)
            cls_name = model.names[cls_id]

            detections.append({
                'class_id': cls_id,
                'class_name': cls_name,
                'conf': score,
                'cx': float(cx), 'cy': float(cy),
                'w': float(w), 'h': float(h),
                'theta_rad': theta,
                'theta_deg': float(np.degrees(theta)),
            })

            box = draw_obb(annotated, cx, cy, w, h, theta)
            label = f"{cls_name} {score:.2f}"
            x0, y0 = int(box[0][0]), int(box[0][1])
            cv2.putText(
                annotated, label, (x0, max(y0 - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    return detections, annotated


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir',
                   default='/home/kluge7/workspaces/isaac_ros-dev/src/pointcloud/data/color')
    p.add_argument('--output-dir',
                   default='/home/kluge7/workspaces/isaac_ros-dev/src/pointcloud/data/yolo')
    p.add_argument('--model', default=os.path.expanduser(
        '~/Downloads/best_valve_handle_real_and_sim.pt'))
    p.add_argument('--conf', type=float, default=0.25)
    p.add_argument('--device', default='cpu')
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--ext', default='png', help='input image extension')
    args = p.parse_args()

    if not os.path.isfile(args.model):
        raise FileNotFoundError(f"Model not found: {args.model}")
    if not os.path.isdir(args.input_dir):
        raise FileNotFoundError(f"Input dir not found: {args.input_dir}")

    annotated_dir = os.path.join(args.output_dir, 'annotated')
    detections_dir = os.path.join(args.output_dir, 'detections')
    os.makedirs(annotated_dir, exist_ok=True)
    os.makedirs(detections_dir, exist_ok=True)

    print(f"Loading model: {args.model} on {args.device}")
    model = YOLO(args.model)
    print(f"Classes: {model.names}")

    files = sorted(glob(os.path.join(args.input_dir, f"*.{args.ext}")))
    if not files:
        print(f"No *.{args.ext} files in {args.input_dir}")
        return

    print(f"Processing {len(files)} images from {args.input_dir} -> {args.output_dir}")

    total_dets = 0
    for i, path in enumerate(files):
        frame = cv2.imread(path, cv2.IMREAD_COLOR)
        if frame is None:
            print(f"  [{i}] failed to read {path}")
            continue

        detections, annotated = process_frame(
            frame, model, args.conf, args.device, args.imgsz)
        total_dets += len(detections)

        stem = os.path.splitext(os.path.basename(path))[0]
        out_img = os.path.join(annotated_dir, f"{stem}.png")
        out_json = os.path.join(detections_dir, f"{stem}.json")

        cv2.imwrite(out_img, annotated)
        with open(out_json, 'w') as f:
            json.dump({
                'source': os.path.basename(path),
                'image_size': {'h': frame.shape[0], 'w': frame.shape[1]},
                'detections': detections,
            }, f, indent=2)

        summary = ', '.join(
            f"{d['class_name']}({d['conf']:.2f})" for d in detections)
        print(f"  [{i+1}/{len(files)}] {os.path.basename(path)}: "
              f"{len(detections)} det [{summary}]")

    print(f"\nDone. {len(files)} images, {total_dets} total detections.")
    print(f"  Annotated images: {annotated_dir}")
    print(f"  Detection JSONs:  {detections_dir}")


if __name__ == '__main__':
    main()
