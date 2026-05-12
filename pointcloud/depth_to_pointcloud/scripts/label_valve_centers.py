#!/usr/bin/env python3
"""Interactive tool: click the valve centre in each depth image.

Layout (side-by-side):
   [ color image (reference) ] [ depth colormap (click here) ]

Per frame, the depth image (right panel) is the one to click on. Coordinates
are recorded in the original cropped-depth pixel space (not the upscaled
display).

Outputs (in workspace):
   src/pointcloud/valve_centers/annotated/frame_XXXXXX.png    depth + marker
   src/pointcloud/valve_centers/centers/frame_XXXXXX.json     {cx, cy, ...}
   src/pointcloud/valve_centers/centers.json                  all-in-one
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
    p.add_argument('--color-dir', default=f'{WORKSPACE}/data/color')
    p.add_argument('--depth-dir', default=f'{WORKSPACE}/data/depth',
                   help='cropped colormap depth (the one you click on)')
    p.add_argument('--output-dir', default=f'{WORKSPACE}/data/valve_centers')
    p.add_argument('--depth-scale', type=float, default=2.0,
                   help='upscale factor for depth display (clicks scaled back)')
    p.add_argument('--start-frame', type=int, default=0,
                   help='resume from this frame index')
    return p.parse_args()


HELP_TEXT = [
    "LEFT CLICK on depth panel: set valve centre",
    "SPACE / n : save & next",
    "p         : previous",
    "s         : skip (no centre)",
    "c         : clear current click",
    "q / ESC   : quit",
]


def render(color_img, depth_cm, click_d, args, frame_name, idx, total, status):
    """Return the composed display image. click_d is in original depth coords."""
    # Resize depth to match color height for side-by-side
    ch, cw = color_img.shape[:2]
    dh, dw = depth_cm.shape[:2]
    scale_to_color = ch / dh
    d_disp_h = ch
    d_disp_w = int(round(dw * scale_to_color))
    depth_disp = cv2.resize(depth_cm, (d_disp_w, d_disp_h),
                            interpolation=cv2.INTER_NEAREST)

    # Draw click marker on depth display
    if click_d is not None:
        u_disp = int(round(click_d[0] * scale_to_color))
        v_disp = int(round(click_d[1] * scale_to_color))
        cv2.drawMarker(depth_disp, (u_disp, v_disp), (0, 255, 0),
                       markerType=cv2.MARKER_CROSS, markerSize=22, thickness=2)
        cv2.circle(depth_disp, (u_disp, v_disp), 4, (255, 255, 255), 1)

    canvas = np.hstack([color_img, depth_disp])

    # Header bar
    header = np.zeros((34, canvas.shape[1], 3), dtype=np.uint8)
    title = f"[{idx+1}/{total}] {frame_name}   click: {click_d if click_d else '-'}   {status}"
    cv2.putText(header, title, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 1, cv2.LINE_AA)

    # Help bar
    help_bar = np.zeros((20 * len(HELP_TEXT) + 6, canvas.shape[1], 3), dtype=np.uint8)
    for i, line in enumerate(HELP_TEXT):
        cv2.putText(help_bar, line, (8, 16 + i * 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (200, 200, 200), 1, cv2.LINE_AA)

    return np.vstack([header, canvas, help_bar]), scale_to_color, cw, d_disp_w


def main():
    args = parse_args()
    annotated_dir = os.path.join(args.output_dir, 'annotated')
    centers_dir = os.path.join(args.output_dir, 'centers')
    summary_path = os.path.join(args.output_dir, 'centers.json')
    os.makedirs(annotated_dir, exist_ok=True)
    os.makedirs(centers_dir, exist_ok=True)

    depth_files = sorted(glob(os.path.join(args.depth_dir, '*.png')))
    if not depth_files:
        print(f"No depth images in {args.depth_dir}")
        return

    # Restore any prior centers (so we can resume)
    summary = {}
    if os.path.isfile(summary_path):
        try:
            summary = json.load(open(summary_path))
        except Exception:
            summary = {}

    win = 'label_valve_centers'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    state = {'click_disp': None, 'scale': 1.0, 'cw': 0, 'd_disp_w': 0}

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        cw = state['cw']
        if x < cw:
            return  # ignore clicks on color panel
        u_depth_disp = x - cw
        v_depth_disp = y - 34  # subtract header height
        if v_depth_disp < 0 or u_depth_disp < 0 or u_depth_disp >= state['d_disp_w']:
            return
        # Convert display coords back to original cropped-depth coords
        u_orig = u_depth_disp / state['scale']
        v_orig = v_depth_disp / state['scale']
        state['click_disp'] = (u_orig, v_orig)

    cv2.setMouseCallback(win, on_mouse)

    idx = max(0, args.start_frame)
    status = ''
    while 0 <= idx < len(depth_files):
        dp = depth_files[idx]
        stem = os.path.splitext(os.path.basename(dp))[0]
        cp = os.path.join(args.color_dir, f"{stem}.png")
        depth_cm = cv2.imread(dp, cv2.IMREAD_COLOR)
        color_img = cv2.imread(cp, cv2.IMREAD_COLOR) if os.path.isfile(cp) else None
        if depth_cm is None or color_img is None:
            print(f"missing image for {stem}, skipping")
            idx += 1
            continue

        # Restore prior click for this frame if it exists
        prior = summary.get(stem)
        if prior and prior.get('cx') is not None:
            state['click_disp'] = (prior['cx'], prior['cy'])
        else:
            state['click_disp'] = None

        while True:
            disp, scale, cw, d_disp_w = render(
                color_img, depth_cm, state['click_disp'], args,
                stem, idx, len(depth_files), status)
            state['scale'] = scale
            state['cw'] = cw
            state['d_disp_w'] = d_disp_w
            cv2.imshow(win, disp)
            k = cv2.waitKey(20) & 0xFF
            if k == 255:
                continue
            if k in (ord('q'), 27):  # quit
                idx = len(depth_files)
                break
            if k in (ord('n'), ord(' ')):  # save and next
                _save(stem, state['click_disp'], depth_cm, scale,
                      annotated_dir, centers_dir, summary, summary_path,
                      os.path.basename(cp), os.path.basename(dp))
                status = 'saved'
                idx += 1
                break
            if k == ord('p'):  # previous (do not save current)
                if idx > 0:
                    idx -= 1
                break
            if k == ord('s'):  # skip with no centre
                _save(stem, None, depth_cm, scale, annotated_dir, centers_dir,
                      summary, summary_path, os.path.basename(cp), os.path.basename(dp))
                status = 'skipped'
                idx += 1
                break
            if k == ord('c'):  # clear current click
                state['click_disp'] = None

    cv2.destroyAllWindows()
    print(f"\nDone. {sum(1 for v in summary.values() if v.get('cx') is not None)}"
          f"/{len(depth_files)} frames have a centre.")
    print(f"  per-frame JSON: {centers_dir}")
    print(f"  combined JSON : {summary_path}")
    print(f"  annotated PNG : {annotated_dir}")


def _save(stem, click, depth_cm, scale, annotated_dir, centers_dir,
          summary, summary_path, source_color, source_depth):
    dh, dw = depth_cm.shape[:2]
    cx = float(click[0]) if click else None
    cy = float(click[1]) if click else None

    annotated = depth_cm.copy()
    if cx is not None:
        cv2.drawMarker(annotated, (int(round(cx)), int(round(cy))),
                       (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.circle(annotated, (int(round(cx)), int(round(cy))), 3, (255, 255, 255), 1)
    cv2.imwrite(os.path.join(annotated_dir, f"{stem}.png"), annotated)

    record = {
        'frame': stem,
        'source_color': source_color,
        'source_depth': source_depth,
        'depth_size': {'h': dh, 'w': dw},
        'cx': cx,
        'cy': cy,
    }
    with open(os.path.join(centers_dir, f"{stem}.json"), 'w') as f:
        json.dump(record, f, indent=2)
    summary[stem] = record
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)


if __name__ == '__main__':
    main()
