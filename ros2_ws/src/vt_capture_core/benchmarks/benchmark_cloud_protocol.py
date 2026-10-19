"""Compare VTPC round-trip CPU cost with optional vectorized validation.

Run with a Python environment containing vt-capture-core and NumPy:
    python benchmarks/benchmark_cloud_protocol.py

This is an in-process synthetic transport benchmark, not a camera throughput,
rendering, disk-write or hardware acceptance test. Both paths validate every
coordinate and serialize/parse the same 50,000-point frame by default.
"""
from __future__ import annotations

import argparse
import json
import platform
import statistics
import struct
import time

from vt_capture_core.spatial import cloud_protocol


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--points', type=int, default=50_000)
    parser.add_argument('--frames', type=int, default=30)
    parser.add_argument('--samples', type=int, default=7)
    args = parser.parse_args()
    if not 1 <= args.points <= cloud_protocol.MAX_POINTS or min(args.frames, args.samples) < 1:
        parser.error('points must fit the protocol limit; frames and samples must be positive')
    numpy = cloud_protocol._numpy
    if numpy is None:
        parser.error('NumPy is required for this comparison (not for vt-capture-core)')
    header = dict(camera_id='synthetic', frame_key='synthetic-frame',
                  source_realtime_ns=1, time_epoch='time', coordinate_epoch='world',
                  geometry_revision='geometry', geometry_sha256='0'*64,
                  point_count=args.points, world_from_color={
                      'translation_m': [0., 0., 0.], 'quaternion_xyzw': [0., 0., 0., 1.]},
                  worker_instance_id='synthetic-worker', cloud_sequence=1)
    xyz = struct.pack('<3f', .25, -.5, 1.25)*args.points
    rgba = bytes((255, 128, 64, 255))*args.points

    def roundtrip():
        return cloud_protocol.unpack_cloud_frame(cloud_protocol.pack_cloud_frame(header, xyz, rgba))

    samples = {'standard_library': [], 'numpy': []}
    try:
        # Alternate paths to reduce bias from frequency scaling and other load.
        for sample in range(args.samples):
            modes = list(samples) if sample % 2 == 0 else list(reversed(samples))
            for mode in modes:
                cloud_protocol._numpy = None if mode == 'standard_library' else numpy
                roundtrip()
                start = time.process_time()
                for _ in range(args.frames):
                    frame = roundtrip()
                elapsed = time.process_time() - start
                assert frame.xyz_bytes == xyz and frame.rgba_bytes == rgba
                samples[mode].append(elapsed/args.frames*1000.)
    finally:
        cloud_protocol._numpy = numpy
    medians = {mode: statistics.median(values) for mode, values in samples.items()}
    print(json.dumps({
        'scope': 'synthetic in-process pack + unpack; CPU time, not hardware acceptance',
        'python': platform.python_version(), 'machine': platform.machine(),
        'numpy': numpy.__version__, 'points': args.points, 'frames_per_sample': args.frames,
        'samples': args.samples, 'median_cpu_ms_per_frame': medians,
        'cpu_ms_per_frame_samples': samples,
        'speedup': medians['standard_library']/medians['numpy'],
    }, indent=2))


if __name__ == '__main__':
    main()
