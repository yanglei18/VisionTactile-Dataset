#!/usr/bin/env python3
"""Replay existing solved parameters through full and measurement-only MANO.

This validates numerical preservation, not anatomical accuracy or live sensor
timing. It reads existing records only and never recalibrates or opens devices.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from vt_fsglove12.mano import ManoBackend


def compare(path, assets, hand):
    import torch
    torch.set_num_threads(1)
    backend = ManoBackend(assets, hand, compact_live_geometry=True)
    errors, measurement_ms, surface_ms = [], [], []
    with Path(path).open() as stream:
        for line in stream:
            pose = json.loads(line)
            if not pose.get('pose_available') or not pose.get('solver_success'):
                continue
            started = time.perf_counter()
            full = backend.render_mano(pose['mano_pose_rotvec'], pose.get('shape_params'))
            full_elapsed = (time.perf_counter() - started) * 1000.
            started = time.perf_counter()
            measured = backend.measure_mano(pose['mano_pose_rotvec'], pose.get('shape_params'))
            elapsed = (time.perf_counter() - started) * 1000.
            errors.append(float(np.max(np.abs(np.asarray(full['joints_m']) - measured['joints_m']))))
            # Report warm-up separately from steady-state inference.
            measurement_ms.append(elapsed)
            surface_ms.append(full_elapsed)
    if not errors:
        raise ValueError('Record contains no valid solved MANO frames')
    def summary(values):
        steady = values[1:] or values
        return dict(first_ms=values[0], median_ms=float(np.median(steady)),
                    p99_ms=float(np.percentile(steady, 99)), max_ms=max(steady))
    return dict(hand=hand, records=str(path), frames=len(errors),
                max_joint_difference_m=max(errors), tolerance_m=1e-6,
                passed=max(errors) <= 1e-6, measurement=summary(measurement_ms),
                surface=summary(surface_ms))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses', required=True)
    parser.add_argument('--assets', required=True)
    parser.add_argument('--hand', choices=('left', 'right'), required=True)
    args = parser.parse_args()
    result = compare(args.poses, args.assets, args.hand)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
