#!/usr/bin/env python3
"""Score the lab's vision models on camera frames recorded in AirSim.

    ~/venvs/vision/bin/python model_eval.py people RUN_DIR --models ~/models
    ~/venvs/vision/bin/python model_eval.py flood  RUN_DIR --models ~/models

The models are the Model Server's (github.com/aykumar21/Drone_LLM, ONNX
exports, run here with onnxruntime on the CPU):
- people: YOLO11s fine-tuned on VisDrone, 1280px, classes 0-1 (pedestrian,
  people) - the server's "robust" human detector;
- flood: ResNet18 flooded/dry classifier (224px, class 0 = flooded) and
  the FloodNet-trained DeepLabv3 water mask (2 classes, argmax).

RUN_DIR is a sim.sh run recorded with CAMERA=WxH (frames/frames.jsonl:
each frame with the drone's true pose). Ground truth comes from the same
run: targets.jsonl (people, sim.sh test-vision) or risk_map.json's river
laid as AirSim water (sim.sh demo-flood, 3m wide). Needs only onnxruntime,
numpy and opencv - no ROS, no PyTorch - so it runs in its own venv beside
AirSim's NumPy 2.
"""

import argparse
import json
import math
import os
import statistics
import sys

import cv2
import numpy as np
import onnxruntime as ort

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'src', 'coordination_node'))
sys.path.insert(0, os.path.join(HERE, '..'))
from coordination_node.flood_grid import (  # noqa: E402
    Camera, FloodRiskAccumulator)
from make_risk_map import point_segment_distance  # noqa: E402

HFOV_DEG = 87.0          # airsim_tool.CAMERA_HFOV_DEG
CAMERA_CLEARANCE_M = 0.24  # airsim_tool: camera height above the reported altitude
TARGET_HEIGHT_M = 0.3    # airsim_tool: a lying person's top surface
RIVER_WIDTH_M = 3.0      # airsim_tool river default, as sim.sh demo-flood lays it
MIN_ALT_M = 2.5          # frames below this are takeoff/landing
MATCH_M = 1.0            # a detection within this of a person found them


def session(path):
    return ort.InferenceSession(path, providers=['CPUExecutionProvider'])


def frames(run):
    with open(os.path.join(run, 'frames', 'frames.jsonl')) as f:
        for line in f:
            fr = json.loads(line)
            true = fr.get('true') or {}
            if true.get('alt', fr['alt']) < MIN_ALT_M:
                continue
            img = cv2.imread(os.path.join(run, 'frames', fr['image']))
            if img is not None:
                yield fr, true, img


def pose(fr, true):
    """(x, y), camera height above ground, yaw - the true pose."""
    return ((true.get('x', fr['x']), true.get('y', fr['y'])),
            true.get('alt', fr['alt']) + CAMERA_CLEARANCE_M,
            true.get('heading', fr['yaw']))


# ---------------------------------------------------------------- people

def yolo(sess, img, size=1280, conf=0.25, iou=0.45, classes=(0, 1)):
    """Ultralytics-export YOLO: letterbox to size, RGB / 255, NCHW. Output
    (1, 4 + classes, anchors): cx, cy, w, h in input pixels, then class
    scores. Returns [(x1, y1, x2, y2, score)] in image pixels."""
    h, w = img.shape[:2]
    scale = size / max(h, w)
    nh, nw = round(h * scale), round(w * scale)
    canvas = np.full((size, size, 3), 114, np.uint8)
    top, left = (size - nh) // 2, (size - nw) // 2
    canvas[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh))
    x = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    out = sess.run(None, {sess.get_inputs()[0].name: x.transpose(2, 0, 1)[None]})[0][0]
    scores = out[4:][list(classes)].max(axis=0)
    keep = scores >= conf
    boxes, scores = out[:4, keep].T, scores[keep]
    if not len(scores):
        return []
    xywh = [[float(cx - bw / 2), float(cy - bh / 2), float(bw), float(bh)]
            for cx, cy, bw, bh in boxes]
    idx = cv2.dnn.NMSBoxes(xywh, scores.tolist(), conf, iou)
    dets = []
    for i in np.array(idx).flatten():
        bx, by, bw, bh = xywh[i]
        x1, y1 = (bx - left) / scale, (by - top) / scale
        dets.append((x1, y1, x1 + bw / scale, y1 + bh / scale, float(scores[i])))
    return dets


def people(args):
    sess = session(os.path.join(args.models, 'yolo11s_visdrone_human_1280.onnx'))
    with open(os.path.join(args.run, 'targets.jsonl')) as f:
        targets = [json.loads(line) for line in f]
    cam = None
    n_frames = in_view = found = false_pos = 0
    frames_with_person = frames_hit = 0
    errors, examples = [], []
    for k, (fr, true, img) in enumerate(frames(args.run)):
        if k % args.stride:
            continue
        cam = cam or Camera(HFOV_DEG, img.shape[1], img.shape[0])
        xy, height, yaw = pose(fr, true)
        visible = []
        for t in targets:
            p = cam.project((t['x'], t['y']), xy, height - TARGET_HEIGHT_M, yaw)
            if p and 0 <= p[0] < cam.width and 0 <= p[1] < cam.height:
                visible.append(t)
        dets = yolo(sess, img, conf=args.conf)
        ground = [cam.ground_point(((d[0] + d[2]) / 2, (d[1] + d[3]) / 2), xy,
                                   height - TARGET_HEIGHT_M, yaw) for d in dets]
        n_frames += 1
        in_view += len(visible)
        hit_any = False
        for t in visible:
            dists = [math.dist(g, (t['x'], t['y'])) for g in ground if g]
            if dists and min(dists) <= MATCH_M:
                found += 1
                hit_any = True
                errors.append(min(dists))
        for g in ground:
            if g is None or all(math.dist(g, (t['x'], t['y'])) > MATCH_M
                                for t in targets):
                false_pos += 1
        if visible:
            frames_with_person += 1
            frames_hit += hit_any
        if dets and len(examples) < 4 and (visible or k % 7 == 0):
            examples.append((fr['image'], img.copy(), dets))
    print(f'frames scored: {n_frames} (stride {args.stride}, alt >= {MIN_ALT_M} m)')
    print(f'person-in-view instances: {in_view}; found within {MATCH_M} m: {found} '
          f'({100 * found / max(1, in_view):.0f}%)')
    print(f'frames with a person in view: {frames_with_person}; with a correct '
          f'detection: {frames_hit} ({100 * frames_hit / max(1, frames_with_person):.0f}%)')
    print(f'false detections (not within {MATCH_M} m of any person): {false_pos} '
          f'in {n_frames} frames')
    if errors:
        print(f'position error of correct detections: median {statistics.median(errors):.2f} m, '
              f'max {max(errors):.2f} m')
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        for name, img, dets in examples:
            for x1, y1, x2, y2, s in dets:
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (214, 120, 42), 2)
                cv2.putText(img, f'{s:.2f}', (int(x1), max(12, int(y1) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (214, 120, 42), 1)
            cv2.imwrite(os.path.join(args.out, 'people_' + name.replace('/', '_')), img)
        print(f'annotated examples in {args.out}')


# ---------------------------------------------------------------- flood

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(3, 1, 1)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(3, 1, 1)
NORMALIZE = False  # --imagenet-norm


def prep(img, size):
    """RGB, 0-1, NCHW - and ImageNet mean/std when NORMALIZE."""
    x = cv2.cvtColor(cv2.resize(img, size), cv2.COLOR_BGR2RGB)
    x = x.astype(np.float32).transpose(2, 0, 1) / 255.0
    if NORMALIZE:
        x = (x - IMAGENET_MEAN) / IMAGENET_STD
    return x[None]


def classify(sess, img):
    logits = sess.run(None, {sess.get_inputs()[0].name: prep(img, (224, 224))})[0][0]
    return int(np.argmax(logits)) == 0  # class 0 = flooded


def water_mask(sess, img, water_class):
    shape = sess.get_inputs()[0].shape
    size = (shape[3], shape[2]) if isinstance(shape[3], int) else (256, 256)
    logits = sess.run(None, {sess.get_inputs()[0].name: prep(img, size)})[0][0]
    mask = (np.argmax(logits, axis=0) == water_class).astype(np.uint8)
    return cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)


def grid(mask, n):
    h, w = mask.shape
    return [[float(mask[r * h // n:(r + 1) * h // n, c * w // n:(c + 1) * w // n].mean())
             for c in range(n)] for r in range(n)]


def flood(args):
    cls = session(os.path.join(args.models, 'flood_resnet18.onnx'))
    seg = session(os.path.join(args.models, 'flood_deeplab_robust.onnx'))
    with open(os.path.join(args.run, 'risk_map.json')) as f:
        river = [tuple(p) for p in json.load(f)['river']]

    def is_water(p):
        return min(point_segment_distance(p, a, b)
                   for a, b in zip(river, river[1:])) <= RIVER_WIDTH_M / 2

    cam = None
    cls_right = cls_total = 0
    rows = []
    for k, (fr, true, img) in enumerate(frames(args.run)):
        if k % args.stride:
            continue
        cam = cam or Camera(HFOV_DEG, img.shape[1], img.shape[0])
        xy, height, yaw = pose(fr, true)
        # Truth for the frame: share of its pixels (sampled) on the river.
        samples = [cam.ground_point((u, v), xy, height, yaw)
                   for v in range(9, cam.height, 18) for u in range(16, cam.width, 32)]
        truth = sum(is_water(p) for p in samples if p) / len(samples)
        flooded = classify(cls, img)
        mask1 = water_mask(seg, img, 1)
        rows.append((fr, truth, flooded, mask1))
        cls_total += 1
        cls_right += flooded == (truth > 0.02)
    # Which class is water: the one whose fraction tracks the truth.
    m1 = [(t, float(m.mean())) for _, t, _, m in rows]
    wet = [f for t, f in m1 if t > 0.2]
    dry = [f for t, f in m1 if t == 0]
    water_class = 1
    if wet and dry and statistics.mean(wet) < statistics.mean(dry):
        water_class = 0
    seen = sum(t > 0.02 for _, t, _, _ in rows)
    print(f'frames scored: {len(rows)}; with river in view: {seen}')
    print(f'water class index: {water_class} (mean class-1 fraction: river frames '
          f'{statistics.mean(wet) if wet else float("nan"):.2f}, dry frames '
          f'{statistics.mean(dry) if dry else float("nan"):.2f})')
    print(f'classifier flooded/dry correct: {cls_right}/{cls_total} '
          f'({100 * cls_right / max(1, cls_total):.0f}%)')
    for name, n, gated in (('4x4 grid, as the server returns', 4, True),
                           ('4x4 grid, mask on every frame', 4, False),
                           ('16x16 grid, mask on every frame', 16, False)):
        acc = FloodRiskAccumulator(cam)
        for fr, truth, flooded, mask1 in rows:
            mask = mask1 if water_class == 1 else 1 - mask1
            ratios = grid(mask, n)
            if gated and not flooded:
                ratios = [[0.0] * n for _ in range(n)]
            elif gated and max(max(r) for r in ratios) < 0.3:
                frac = float(mask.mean())
                ratios = [[frac] * n for _ in range(n)]
            xy, height, yaw = pose(fr, fr.get('true') or {})
            acc.observe(ratios, xy, height, yaw)
        risk = acc.risk()
        water = [r for c, r in risk.items() if is_water((c[0] + 0.5, c[1] + 0.5))]
        dryc = [r for c, r in risk.items() if not is_water((c[0] + 0.5, c[1] + 0.5))]
        pred = {c for c, r in risk.items() if r >= 0.5}
        true_cells = {c for c in risk if is_water((c[0] + 0.5, c[1] + 0.5))}
        iou = len(pred & true_cells) / max(1, len(pred | true_cells))
        print(f'{name}: cells observed {len(risk)} (river {len(water)}); mean risk river '
              f'{statistics.mean(water) if water else float("nan"):.2f} / dry '
              f'{statistics.mean(dryc) if dryc else float("nan"):.2f}; '
              f'IoU (risk >= 0.5) {iou:.2f}')
        if args.out and n == 16:
            os.makedirs(args.out, exist_ok=True)
            path = os.path.join(args.out, 'risk_map_from_model.json')
            with open(path, 'w') as f:
                json.dump(acc.to_risk_map('deeplab_robust 16x16, ' + args.run), f)
            print(f'risk map written: {path}')
    if args.out:
        for fr, truth, flooded, mask1 in rows:
            if truth > 0.25:
                img = cv2.imread(os.path.join(args.run, 'frames', fr['image']))
                mask = mask1 if water_class == 1 else 1 - mask1
                tint = img.copy()
                tint[mask > 0] = (0.5 * tint[mask > 0] + [110, 60, 0]).astype(np.uint8)
                cv2.imwrite(os.path.join(args.out, 'flood_' + fr['image'].replace('/', '_')),
                            np.hstack([img, tint]))
                break


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('what', choices=('people', 'flood'))
    p.add_argument('run', help='sim.sh run directory with frames/')
    p.add_argument('--models', default=os.path.expanduser('~/models'))
    p.add_argument('--stride', type=int, default=1, help='score every Nth frame')
    p.add_argument('--conf', type=float, default=0.25, help='person score threshold')
    p.add_argument('--out', help='write annotated examples / risk map here')
    p.add_argument('--imagenet-norm', action='store_true',
                   help='flood models: normalise with ImageNet mean/std (the '
                        "server's own scripts only scale to 0-1)")
    a = p.parse_args()
    global NORMALIZE
    NORMALIZE = a.imagenet_norm
    people(a) if a.what == 'people' else flood(a)


if __name__ == '__main__':
    main()
