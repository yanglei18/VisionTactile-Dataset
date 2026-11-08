"""Verify an explicitly selected numerical environment without device access.

The probe runs the same resource and model loaders as the hand service. It
does not construct a controller, open a serial source, or alter calibration.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import tempfile


_PROBE = r'''
import contextlib, json, sys
from pathlib import Path
root, bundle, hand = Path(sys.argv[1]).resolve(), Path(sys.argv[2]), sys.argv[3]
with contextlib.redirect_stdout(sys.stderr):
    import vt_fsglove12
    from vt_fsglove12 import hand_service, teaching_geometry
    from vt_fsglove12.hand_service_resources import load_service_resources
    from vt_fsglove12.upstream_core import load_upstream_core
    from vt_fsglove12.mano import ManoBackend
    import numpy, scipy, vqf, qmt, torch
    expected = root / 'tools/fsglove12/src/vt_fsglove12'
    for module in (vt_fsglove12, hand_service, teaching_geometry):
        if not Path(module.__file__).resolve().is_relative_to(expected):
            raise RuntimeError('Numerical environment imports another checkout: ' + module.__file__)
    resources = load_service_resources(hand=hand, resources=bundle)
    core = load_upstream_core(resources['upstream_checkout'], resources['mano_assets'])
    core.require_model(hand)
    frame = ManoBackend(resources['mano_assets'], hand=hand, compact_live_geometry=True).render(numpy.zeros((16, 3)))
    if not numpy.isfinite(frame['vertices_m']).all():
        raise ValueError('Model produced non-finite neutral geometry')
print(json.dumps(dict(schema_version=1, hand=hand, source_root=str(root),
    module_path=str(Path(vt_fsglove12.__file__).resolve()), version=vt_fsglove12.__version__,
    python=str(Path(sys.executable).absolute()), vertices=len(frame['vertices_m']),
    hardware_accessed=False, controller_started=False), allow_nan=False))
'''


def probe_hand_runtime(interpreter, resources, hand, repo_root, *, timeout=45.):
    """Check source binding, dependencies, bundle semantics and actual MANO load."""
    python = Path(interpreter).expanduser().absolute()
    if not python.is_file() or not os.access(python, os.X_OK):
        raise ValueError("Numerical Python must be an existing executable")
    if hand not in {"left", "right"}:
        raise ValueError("Unknown hand")
    environment = {key: value for key, value in os.environ.items() if key not in {
        "PYTHONPATH", "PYTHONHOME", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "LD_LIBRARY_PATH"}}
    environment.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    with tempfile.TemporaryDirectory(prefix="vt-hand-preflight-") as temporary:
        with (Path(temporary) / "stdout").open("w+b") as output, (Path(temporary) / "stderr").open("w+b") as errors:
            try:
                result = subprocess.run([str(python), "-I", "-c", _PROBE,
                    str(Path(repo_root).resolve()), str(Path(resources).resolve()), hand],
                    stdin=subprocess.DEVNULL, stdout=output, stderr=errors,
                    env=environment, timeout=timeout, check=False)
            except subprocess.TimeoutExpired as exc:
                raise ValueError(f"{hand} numerical environment verification timed out") from exc
            if result.returncode:
                errors.seek(0)
                raise ValueError(f"{hand} numerical environment verification failed: " +
                                 errors.read(8192).decode("utf-8", errors="replace"))
            output.seek(0)
            payload = output.read(16385)
    if len(payload) > 16384:
        raise ValueError("Numerical environment response exceeds its limit")
    try:
        evidence = json.loads(payload)
    except (ValueError, UnicodeError) as exc:
        raise ValueError("Numerical environment returned invalid evidence") from exc
    if (not isinstance(evidence, dict) or evidence.get("schema_version") != 1
            or evidence.get("hand") != hand or evidence.get("hardware_accessed") is not False
            or evidence.get("controller_started") is not False
            or evidence.get("source_root") != str(Path(repo_root).resolve())):
        raise ValueError("Numerical environment identity did not match this candidate")
    return evidence
