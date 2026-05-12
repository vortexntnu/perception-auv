#!/usr/bin/env python3
"""Map YOLO-OBB detections from the color image to the cropped depth image
using camera intrinsics + the depth->color extrinsic.

For each OBB corner (u_c, v_c) in color, with a depth Z (metres):
    X_c = (u_c - cx_c) * Z / fx_c
    Y_c = (v_c - cy_c) * Z / fy_c
    (X_d, Y_d, Z_d) = (X_c, Y_c, Z) - T_depth_to_color
    u_d = fx_d * X_d / Z_d + cx_d
    v_d = fy_d * Y_d / Z_d + cy_d
    (u_d_cropped, v_d_cropped) = (u_d - x_offset, v_d - y_offset)

Z is sampled from the cropped raw depth (16UC1, mm) at the OBB centre, using
the previous frame's crude estimate as a seed; falls back to --default-z when
the depth pixel is invalid.
"""

import argparse
import json
import os
from glob import glob

import cv2
import numpy as np

WORKSPACE = '/home/kluge7/workspaces/isaac_ros-dev/src/pointcloud'


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--color-detections', default=f'{WORKSPACE}/data/yolo/detections')
    p.add_argument('--depth-images-colormap', default=f'{WORKSPACE}/data/depth',
                   help='cropped colormap depth (for visualization background)')
    p.add_argument('--depth-images-raw', default=f'{WORKSPACE}/data/depth_raw',
                   help='cropped raw 16UC1 depth (mm); needed for Z lookup')
    p.add_argument('--output-dir', default=f'{WORKSPACE}/data/depth_bbox_mapped')
    # Crop ROI (same as depth_cropper params)
    p.add_argument('--x-offset', type=int, default=260)
    p.add_argument('--y-offset', type=int, default=190)
    # Color intrinsics from user-supplied calibration yaml (896x504, same as data)
    p.add_argument('--color-fx', type=float, default=687.47)
    p.add_argument('--color-fy', type=float, default=572.95)
    p.add_argument('--color-cx', type=float, default=410.31)
    p.add_argument('--color-cy', type=float, default=252.74)
    # Depth intrinsics
    p.add_argument('--depth-fx', type=float, default=449.44970703125)
    p.add_argument('--depth-fy', type=float, default=449.44970703125)
    p.add_argument('--depth-cx', type=float, default=444.8199462890625)
    p.add_argument('--depth-cy', type=float, default=248.22657775878906)
    # Color distortion (plumb_bob: k1, k2, p1, p2, k3) — from user calibration yaml
    p.add_argument('--color-d', nargs=5, type=float,
                   default=[0.268120, 0.659922, 0.004430, -0.024228, -0.353035],
                   metavar=('k1', 'k2', 'p1', 'p2', 'k3'))
    # Extrinsic: pose of color in depth frame (translation, metres)
    p.add_argument('--depth-to-color-tx', type=float, default=-0.059)
    p.add_argument('--depth-to-color-ty', type=float, default=0.0)
    p.add_argument('--depth-to-color-tz', type=float, default=0.0)
    p.add_argument('--default-z', type=float, default=0.9,
                   help='fallback Z [m] when raw depth is invalid at sample. '
                        '0.9 m empirically minimises mean residual against '
                        'hand-labelled valve centres on this bag.')
    p.add_argument('--classes', default='',
                   help='comma-separated class names to keep (empty = all)')
    return p.parse_args()


def project_color_to_depth(uv_color, Z, args, K_color, D_color):
    """Project a (distorted) color pixel + Z [m] to a depth-image pixel.

    Uses cv2.undistortPoints to remove color radial/tangential distortion before
    back-projecting into 3D. The depth image is rectified (d=0), so the forward
    projection is a plain pinhole.
    """
    pts = np.array([[uv_color]], dtype=np.float64)  # shape (1, 1, 2)
    # Returns normalized coords (x' = X/Z, y' = Y/Z) in the color frame.
    norm = cv2.undistortPoints(pts, K_color, D_color)
    x_n, y_n = float(norm[0, 0, 0]), float(norm[0, 0, 1])

    X_c = x_n * Z
    Y_c = y_n * Z
    # depth_to_color is the pose of color in depth, so color->depth
    # subtracts the translation: P_d = P_c - T_depth_to_color
    X_d = X_c - args.depth_to_color_tx
    Y_d = Y_c - args.depth_to_color_ty
    Z_d = Z - args.depth_to_color_tz
    if Z_d <= 0:
        return None
    u_d = args.depth_fx * X_d / Z_d + args.depth_cx
    v_d = args.depth_fy * Y_d / Z_d + args.depth_cy
    return float(u_d), float(v_d)


def obb_corners(cx, cy, w, h, theta_rad):
    rect = ((float(cx), float(cy)), (float(w), float(h)),
            float(np.degrees(theta_rad)))
    return cv2.boxPoints(rect)


def sample_z_meters(depth_raw_cropped, u_d_cropped, v_d_cropped, win=5):
    """Median of valid depth values in a window around (u, v); returns metres."""
    h, w = depth_raw_cropped.shape[:2]
    x = int(round(u_d_cropped))
    y = int(round(v_d_cropped))
    if not (0 <= x < w and 0 <= y < h):
        return None
    x0, x1 = max(0, x - win), min(w, x + win + 1)
    y0, y1 = max(0, y - win), min(h, y + win + 1)
    patch = depth_raw_cropped[y0:y1, x0:x1]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 1000.0  # mm -> m


def map_obb(det, depth_raw_cropped, args, K_color, D_color):
    """Project one color OBB into cropped-depth coords. Returns dict or None."""
    cx_c, cy_c = det['cx'], det['cy']
    w, h, theta = det['w'], det['h'], det['theta_rad']

    # 1) Initial Z guess via a crude (offset-only) sample of raw depth.
    init_u = cx_c - args.x_offset
    init_v = cy_c - args.y_offset
    Z = sample_z_meters(depth_raw_cropped, init_u, init_v) or args.default_z

    # 2) Project centre with that Z.
    uv = project_color_to_depth((cx_c, cy_c), Z, args, K_color, D_color)
    if uv is None:
        return None
    u_d_full, v_d_full = uv

    # 3) Refine Z by sampling at the projected pixel (one iteration).
    u_crop = u_d_full - args.x_offset
    v_crop = v_d_full - args.y_offset
    Z2 = sample_z_meters(depth_raw_cropped, u_crop, v_crop)
    if Z2 is not None and abs(Z2 - Z) > 0.02:
        Z = Z2
        uv = project_color_to_depth((cx_c, cy_c), Z, args, K_color, D_color)
        if uv is None:
            return None
        u_d_full, v_d_full = uv
        u_crop = u_d_full - args.x_offset
        v_crop = v_d_full - args.y_offset

    # 4) Project each color corner separately so undistortion + intrinsics
    #    differences (scale/shear) are captured per-corner.
    corners_color = obb_corners(cx_c, cy_c, w, h, theta)
    corners_depth = []
    for u, v in corners_color:
        uv_d = project_color_to_depth((float(u), float(v)), Z, args, K_color, D_color)
        if uv_d is None:
            return None
        corners_depth.append((uv_d[0] - args.x_offset, uv_d[1] - args.y_offset))
    corners_depth = np.array(corners_depth, dtype=np.float32)

    return {
        'class_id': det['class_id'],
        'class_name': det['class_name'],
        'conf': det['conf'],
        'Z_m': Z,
        'cx_depth_cropped': u_crop,
        'cy_depth_cropped': v_crop,
        'corners_depth_cropped': corners_depth.tolist(),
        # For convenience, also fit an axis-aligned-ish OBB summary
        'w': float(det['w']),
        'h': float(det['h']),
        'theta_rad': float(det['theta_rad']),
    }


def draw_polygon(img, pts, color, label):
    pts_i = np.intp(pts)
    cv2.polylines(img, [pts_i], True, color, 2)
    x0, y0 = int(pts_i[0][0]), int(pts_i[0][1])
    cv2.putText(img, label, (x0, max(y0 - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def main():
    args = parse_args()
    keep_classes = {c.strip() for c in args.classes.split(',') if c.strip()}

    K_color = np.array([[args.color_fx, 0.0,            args.color_cx],
                        [0.0,           args.color_fy,  args.color_cy],
                        [0.0,           0.0,            1.0]], dtype=np.float64)
    D_color = np.array(args.color_d, dtype=np.float64)

    have_raw = os.path.isdir(args.depth_images_raw)
    if not have_raw:
        print(f"NOTE: raw depth dir not found ({args.depth_images_raw}). "
              f"Falling back to a fixed Z = {args.default_z} m for every OBB. "
              f"Re-run sync_and_save_node to enable per-pixel Z sampling.")

    annotated_dir = os.path.join(args.output_dir, 'annotated')
    detections_dir = os.path.join(args.output_dir, 'detections')
    os.makedirs(annotated_dir, exist_ok=True)
    os.makedirs(detections_dir, exist_ok=True)

    palette = {'valve': (0, 255, 0), 'handle': (0, 165, 255)}
    default_color = (255, 255, 255)

    json_files = sorted(glob(os.path.join(args.color_detections, '*.json')))
    if not json_files:
        print(f"No JSONs in {args.color_detections}")
        return

    n_pairs = n_dets = n_centre_in = 0
    for jf in json_files:
        stem = os.path.splitext(os.path.basename(jf))[0]
        cm_path = os.path.join(args.depth_images_colormap, f"{stem}.png")
        raw_path = os.path.join(args.depth_images_raw, f"{stem}.png")
        if not os.path.isfile(cm_path):
            continue
        cm_img = cv2.imread(cm_path, cv2.IMREAD_COLOR)
        if cm_img is None:
            continue
        if have_raw and os.path.isfile(raw_path):
            depth_raw = cv2.imread(raw_path, cv2.IMREAD_UNCHANGED)
        else:
            # zeros => sample_z_meters returns None => map_obb falls back to default-z
            depth_raw = np.zeros(cm_img.shape[:2], dtype=np.uint16)
        dh, dw = cm_img.shape[:2]

        with open(jf) as f:
            color_doc = json.load(f)

        mapped = []
        for det in color_doc.get('detections', []):
            if keep_classes and det['class_name'] not in keep_classes:
                continue
            m = map_obb(det, depth_raw, args, K_color, D_color)
            if m is None:
                continue
            mapped.append(m)
            if 0 <= m['cx_depth_cropped'] < dw and 0 <= m['cy_depth_cropped'] < dh:
                n_centre_in += 1
            color = palette.get(m['class_name'], default_color)
            draw_polygon(cm_img, np.array(m['corners_depth_cropped']),
                         color, f"{m['class_name']} {m['conf']:.2f} z={m['Z_m']:.2f}m")

        cv2.imwrite(os.path.join(annotated_dir, f"{stem}.png"), cm_img)
        with open(os.path.join(detections_dir, f"{stem}.json"), 'w') as f:
            json.dump({
                'source_color_detections': os.path.basename(jf),
                'source_depth_colormap': os.path.basename(cm_path),
                'source_depth_raw': os.path.basename(raw_path),
                'depth_size': {'h': dh, 'w': dw},
                'roi_offset': {'x': args.x_offset, 'y': args.y_offset},
                'intrinsics_color': {
                    'fx': args.color_fx, 'fy': args.color_fy,
                    'cx': args.color_cx, 'cy': args.color_cy},
                'intrinsics_depth': {
                    'fx': args.depth_fx, 'fy': args.depth_fy,
                    'cx': args.depth_cx, 'cy': args.depth_cy},
                'depth_to_color_translation': [args.depth_to_color_tx,
                                               args.depth_to_color_ty,
                                               args.depth_to_color_tz],
                'detections': mapped,
            }, f, indent=2)

        n_pairs += 1
        n_dets += len(mapped)

    print(f"Processed {n_pairs} pairs; mapped {n_dets} detections "
          f"({n_centre_in} with centre inside the cropped depth).")
    print(f"  Annotated: {annotated_dir}")
    print(f"  JSON:      {detections_dir}")


if __name__ == '__main__':
    main()
