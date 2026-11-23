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
import contextlib, importlib.util, json, site, sys
from pathlib import Path
root = Path(sys.argv[1]).resolve() if sys.argv[1] else None
bundle, hand = sys.argv[2], sys.argv[3]
parameters = json.loads(sys.argv[4]) if len(sys.argv) > 4 else {}
def identity(path):
    path = Path(path).absolute()
    resolved = path.resolve(strict=True)
    info = resolved.stat()
    if not resolved.is_file():
        raise ValueError('Preflight expected a regular file: ' + str(path))
    return [str(resolved), info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns]
checked = {}
def bind(path):
    path = str(Path(path).absolute())
    checked[path] = identity(path)
spec = importlib.util.find_spec('vt_fsglove12')
if spec is None or spec.origin is None:
    raise ValueError('Numerical environment has no hand runtime')
package = Path(spec.origin).resolve().parent
if root is not None and package != root / 'tools/fsglove12/src/vt_fsglove12':
    raise ValueError('Numerical environment imports another checkout: ' + str(package))
for path in package.rglob('*.py'):
    bind(path)
for location in site.getsitepackages():
    for pattern in ('*.pth', '__editable__*.py'):
        for path in Path(location).glob(pattern):
            bind(path)
for path in (Path(sys.prefix) / 'pyvenv.cfg', Path(sys.executable).absolute()):
    if path.is_file():
        bind(path)
if len(checked) > 2048:
    raise ValueError('Numerical source inventory exceeds its limit')
with contextlib.redirect_stdout(sys.stderr):
    import vt_fsglove12
    from vt_fsglove12 import hand_service, teaching_geometry
    from vt_fsglove12.hand_service_resources import load_service_resources
    from vt_fsglove12.upstream_core import load_upstream_core
    from vt_fsglove12.mano import ManoBackend
    import numpy, scipy, vqf, qmt, torch
    for module in (vt_fsglove12, hand_service, teaching_geometry):
        if not Path(module.__file__).resolve().is_relative_to(package):
            raise RuntimeError('Numerical environment imports another checkout: ' + module.__file__)
    resources = load_service_resources(hand=hand, resources=bundle or None, **parameters)
    core = load_upstream_core(resources['upstream_checkout'], resources['mano_assets'])
    core.require_model(hand)
    frame = ManoBackend(resources['mano_assets'], hand=hand, compact_live_geometry=True).render(numpy.zeros((16, 3)))
    if not numpy.isfinite(frame['vertices_m']).all():
        raise ValueError('Model produced non-finite neutral geometry')
if any(identity(path) != stamp for path, stamp in checked.items()):
    raise ValueError('Numerical sources changed during verification; retry preflight')
print(json.dumps(dict(schema_version=1, hand=hand, source_root=str(root) if root else '',
    module_path=str(Path(vt_fsglove12.__file__).resolve()), version=vt_fsglove12.__version__,
    python=str(Path(sys.executable).absolute()), vertices=len(frame['vertices_m']),
    checked_files=checked,
    hardware_accessed=False, controller_started=False), allow_nan=False))
'''


def probe_hand_runtime(interpreter, resources, hand, repo_root=None, *, explicit=None, output_rate_hz=None, timeout=45.):
    """Check source binding, dependencies, bundle semantics and actual MANO load."""
    python = Path(interpreter).expanduser().absolute()
    if not python.is_file() or not os.access(python, os.X_OK):
        raise ValueError("Numerical Python must be an existing executable")
    if hand not in {"left", "right"}:
        raise ValueError("Unknown hand")
    parameters = dict(explicit or {})
    if output_rate_hz is not None:
        parameters["output_rate_hz"] = float(output_rate_hz)
    root = str(Path(repo_root).resolve()) if repo_root is not None else ""
    command = [str(python), "-I", "-c", _PROBE, root,
               str(Path(resources).resolve()) if resources else "", hand]
    if parameters:
        command.append(json.dumps(parameters, allow_nan=False))
    environment = {key: value for key, value in os.environ.items() if key not in {
        "PYTHONPATH", "PYTHONHOME", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "LD_LIBRARY_PATH"}}
    environment.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    with tempfile.TemporaryDirectory(prefix="vt-hand-preflight-") as temporary:
        with (Path(temporary) / "stdout").open("w+b") as output, (Path(temporary) / "stderr").open("w+b") as errors:
            try:
                result = subprocess.run(command,
                    stdin=subprocess.DEVNULL, stdout=output, stderr=errors,
                    env=environment, timeout=timeout, check=False)
            except subprocess.TimeoutExpired as exc:
                raise ValueError(f"{hand} numerical environment verification timed out") from exc
            if result.returncode:
                errors.seek(0)
                raise ValueError(f"{hand} numerical environment verification failed: " +
                                 errors.read(8192).decode("utf-8", errors="replace"))
            output.seek(0)
            payload = output.read(1024 * 1024 + 1)
    if len(payload) > 1024 * 1024:
        raise ValueError("Numerical environment response exceeds its limit")
    try:
        evidence = json.loads(payload)
    except (ValueError, UnicodeError) as exc:
        raise ValueError("Numerical environment returned invalid evidence") from exc
    if (not isinstance(evidence, dict) or evidence.get("schema_version") != 1
            or evidence.get("hand") != hand or evidence.get("hardware_accessed") is not False
            or evidence.get("controller_started") is not False
            or evidence.get("source_root") != root):
        raise ValueError("Numerical environment identity did not match this candidate")
    return evidence
