"""Prepare a private portable runtime using the existing trusted MANO environment."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import tempfile

from .resources import sha256


def prepare(output, *, mano_assets, fsglove_source, config, response_profile, license_file):
    import numpy as np
    from vt_fsglove12.mano import ManoBackend
    from vt_fsglove12.config import load_config
    from vt_fsglove12.guided_calibration import model_motion_axes
    from vt_fsglove12.upstream_core import inspect_upstream_sources
    from vt_capture_console.backend.tactile_reference_store import read_reference_profile

    output = Path(output).expanduser().absolute()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite resource directory: {output}")
    configuration = load_config(config)
    if configuration["hand"] != "right":
        raise ValueError("First release supports the right hand")
    provenance = inspect_upstream_sources(fsglove_source)
    reference = read_reference_profile(response_profile)
    license_file = Path(license_file)
    if not license_file.is_file() or not license_file.read_text(encoding="utf-8").strip():
        raise ValueError("A model-use notice is required")
    backend = ManoBackend(mano_assets, hand="right")
    rest = backend.export_rest()
    geometry = backend.render(np.zeros((16, 3)))
    axes = model_motion_axes(geometry["joints_m"], hand="right")
    # Retain numerical provenance without baking the maintainer's paths into a delivery.
    for value in (rest, geometry):
        value["provenance"] = {key: item for key, item in value.get("provenance", {}).items()
                               if not key.endswith(("path", "root", "file"))}
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".hand-runtime-", dir=output.parent))
    try:
        def write(name, value):
            (stage / name).write_text(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
        write("hand.json", dict(schema_version=1, neutral_geometry=geometry, model_axes=axes))
        write("rest.json", rest)
        write("config.json", configuration)
        write("default_response.json", reference.to_dict())
        (stage / "mano/models").mkdir(parents=True)
        shutil.copyfile(Path(mano_assets) / "models/MANO_RIGHT.pkl", stage / "mano/models/MANO_RIGHT.pkl")
        for record in provenance["sources"].values():
            target = stage / "upstream" / record["relative_path"]
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(record["path"], target)
        for name in ("LICENSE", "LICENSE.md", "LICENSE.txt"):
            source = Path(fsglove_source) / name
            if source.is_file():
                shutil.copyfile(source, stage / "FSGLOVE_LICENSE.txt")
                break
        shutil.copyfile(license_file, stage / "MODEL_LICENSE.txt")
        files = {path.relative_to(stage).as_posix(): sha256(path) for path in sorted(stage.rglob("*")) if path.is_file()}
        write("manifest.json", dict(schema_version=1, product="hand-motion-touch", files=files))
        stage.rename(output)
    finally:
        if stage.exists():
            shutil.rmtree(stage)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("output", "mano-assets", "fsglove-source", "config", "response-profile", "license-file"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = vars(parser.parse_args(argv))
    print(prepare(**args))


if __name__ == "__main__":
    main()
