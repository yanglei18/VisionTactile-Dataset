"""Recover an orthogonal cube's printed face layout from one uncalibrated video.

A perspective transform takes the four decoded print corners to their physical
face in a pinhole image. Extrapolating from the centred 24 mm print to the 30 mm
face predicts the shared edge, including its direction. No camera-to-hand
transform or metric trajectory is estimated by this importer.
"""
import argparse
from collections import Counter, defaultdict
from itertools import combinations, product
import json
from pathlib import Path

import cv2
import numpy as np

from .cube import CubeDetector, CubeLayout

_SQUARE = np.float32([[-1, -1], [1, -1], [1, 1], [-1, 1]])
_FACES = ('top', 'bottom', 'left', 'right', 'front', 'back')


def _seam(a, b):
    # Edges on adjacent outward-facing surfaces wind in opposite directions.
    a0, a1 = a[:, None, :], np.roll(a, -1, axis=0)[:, None, :]
    b0, b1 = b[None, :, :], np.roll(b, -1, axis=0)[None, :, :]
    scale = np.minimum(np.linalg.norm(a0-a1, axis=2), np.linalg.norm(b0-b1, axis=2))
    error = np.sqrt(((a0-b1)**2 + (a1-b0)**2).sum(axis=2) / 2) / np.maximum(scale, 1.)
    best, other = np.argsort(error.ravel())[:2]
    if error.flat[best] > .13 or error.flat[other]-error.flat[best] < .25:
        return None
    return tuple(int(x) for x in np.unravel_index(best, error.shape))


def _face_corners(tag, face, corner):
    pts = CubeLayout(((tag, face, corner),)).object_points()[tag]
    mid = pts.mean(axis=0)
    return mid + (pts-mid) * (CubeLayout.cube_edge_m/CubeLayout.tag_edge_m)


def _assemble(votes):
    known = {tag: (name, quarter) for tag, name, quarter in CubeLayout.default().faces}
    confirmed = []
    for (a, b), counter in votes.items():
        good = counter.most_common()
        if good[0][1] < 3:
            continue
        if len(good) > 1 and good[1][1] >= 2:
            raise ValueError(f'Inconsistent printed edges for IDs {a}, {b}')
        j, k = good[0][0]
        confirmed.append((a, b, j, k))
    def edge(tag, index):
        return _face_corners(tag, *known[tag])[[index, (index+1) % 4]]
    while True:
        n = len(known)
        for a, b, j, k in confirmed:
            if b in known and a not in known:
                a, b, j, k = b, a, k, j
            if a not in known:
                continue
            if b in known:
                if not np.allclose(edge(a, j), edge(b, k)[::-1], atol=1e-7):
                    raise ValueError(f'Inconsistent cube geometry at IDs {a}, {b}')
                continue
            choices = []
            for face, corner in product(_FACES, range(4)):
                if any(f == face for f, _ in known.values()):
                    continue
                other = _face_corners(b, face, corner)[[(k+1) % 4, k]]
                if np.allclose(edge(a, j), other, atol=1e-7):
                    choices.append((face, corner))
            if len(choices) != 1:
                raise ValueError(f'No unique placement for ID {b} next to ID {a}')
            known[b] = choices[0]
        if n == len(known):
            break
    seen = {tag for a,b,_,_ in confirmed for tag in (a,b)}
    if len(known) != 6 or len(seen) != 6:
        raise ValueError('Need all six IDs in connected, adjacent views of one cube')
    return CubeLayout(tuple((tag, *known[tag]) for tag in sorted(known)))


def analyze(path):
    detector = CubeDetector(CubeLayout.default())
    count, frames_with_tags = 0, 0
    counts = Counter()
    folds = [defaultdict(Counter), defaultdict(Counter)]
    cap = cv2.VideoCapture(str(path))
    try:
        if not cap.isOpened():
            raise ValueError(f'Cannot open video {path}')
        fps = cap.get(cv2.CAP_PROP_FPS)
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            sightings = detector.read_corners(frame)
            # Alternating short frame blocks keep fit and validation separate.
            votes = folds[(count // 5) % 2]
            count += 1
            if len(sightings) > 3 or len(dict(sightings)) != len(sightings):
                raise ValueError('More than one cube in layout video')
            if sightings:
                frames_with_tags += 1
            counts.update(t for t, _ in sightings)
            faces = {}
            for tag, points in sightings:
                warp = cv2.getPerspectiveTransform(_SQUARE, np.asarray(points, dtype=np.float32))
                outside = _SQUARE * (CubeLayout.cube_edge_m/CubeLayout.tag_edge_m)
                faces[tag] = cv2.perspectiveTransform(outside[None, :, :], warp)[0]
            for a, b in combinations(sorted(faces), 2):
                pair = _seam(faces[a], faces[b])
                if pair is not None:
                    votes[a,b][pair] += 1
    finally:
        cap.release()
    first, second = (_assemble(v) for v in folds)
    if first != second:
        raise ValueError('Independent frame groups give different face layouts')
    return first, dict(frames=count, frames_with_tags=frames_with_tags, fps=fps,
                       detections=dict(sorted(counts.items())), layout_sha256=first.sha256,
                       independent_frame_groups=2)


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument('video', type=Path)
    args.add_argument('output', type=Path, help='Create a new private layout JSON file')
    opt = args.parse_args()
    try:
        layout, evidence = analyze(opt.video)
        layout.save(opt.output)
        print(json.dumps(evidence, indent=2))
    except (ValueError, OSError, cv2.error) as exc:
        args.exit(1, str(exc)+'\n')


if __name__ == '__main__':
    main()
