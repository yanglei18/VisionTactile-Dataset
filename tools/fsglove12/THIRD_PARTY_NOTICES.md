# Third-party boundaries

The repository license applies to independently authored adapter files only.
This package is not an official FSGlove release and is not a redistribution of
FSGlove source, MANO model files, or a complete DiffHCal implementation.

- FSGlove is pinned to commit
  `a7988f051c35f5ee6157353ee4022d73aa1e4cc8`. The `upstream-check`,
  `upstream-live`, and `upstream-shape` modes load numerical definitions from a
  separately obtained, trusted external checkout. The loader verifies four
  source-file SHA-256 digests and compiles selected original AST definitions
  without rewriting their numerical bodies. Only dependency wiring, explicit
  MANO asset paths, and isolated random seeding are adapted. The source is not
  copied into or distributed with this package; the official GUI, network
  receivers, and hardware state machine are not started. A public GitHub URL
  does not by itself establish permission to redistribute that source.
- Official-mode execution is distinct from the legacy independently authored
  `live`/`calibrate`/`solve` algorithms. `upstream-live` uses the original
  `Raw2MANOCalibrator`; the sparse-input wrapper is new adapter code, not an
  upstream 12-IMU release. New v2 profiles use an independently authored,
  explicitly labelled missing-DIP prior: a missing four-finger DIP follows
  current observed PIP flexion, default ratio 0.65 and upper bound 1.65 radians,
  only while both required proximal/middle segments are observed. Missing
  dependencies fall back to neutral; missing thumb CMC and other slots remain
  neutral. Measured inputs, including measured DIPs, are never overwritten.
  Ratios and bounds are configurable empirical assumptions, not upstream
  algorithms or anatomical measurements. Original v1 profiles retain their
  fixed-zero policy and are not silently migrated. The wrapper algebraically
  creates virtual inputs for those declared priors and still calls the original
  official inference; neither virtual inputs nor coupled angles may be described
  as sensor observations. The official touch-contact optimizer used by `upstream-shape`
  optimizes MANO beta only; the public installation `CALC` branch remains a
  TODO. Neither mode establishes a complete implementation of every algorithm
  described in the paper or equivalent physical accuracy.
- VQF, NumPy and SciPy are separately installed dependencies. Preserve the
  licenses/notices of the versions distributed with any application.
- The independent optional `functional` extra pins `qmt==0.2.4`. The new
  official-mode functional adapter actually calls `qmt.quatProject` from
  [`qmt/functions/quaternion.py`](https://github.com/dlaidig/qmt/blob/0fa8d32eb461e14d78e9ddbd569664ea59bcea19/qmt/functions/quaternion.py),
  whose file header identifies copyright 2021 Daniel Laidig and the MIT license.
  The reviewed file SHA-256 is
  `0f698376e0a11acb16a9efaef30fc299bc62994390a809d302d7ce3eaa3c959e`;
  functional profiles additionally record the actually executed package version,
  source path and source-file digest. QMT is installed separately, not vendored
  into this tool. Preserve the applicable copyright and license notices when
  distributing dependencies. QMT has file-level exceptions: in particular,
  `joint_axis_est_hinge_olsson.py` is marked `LicenseRef-Unspecified`, not MIT.
  This adapter neither calls nor copies `jointAxisEstHingeOlsson`; the package's
  general MIT label must not be used to relicense that file or every dependency.
  Earlier isolated Olsson reference experiments in `tools/glove_hand_pose/`
  are separate from this runtime path.
- Functional-axis fitting uses independently authored SciPy/NumPy/SVD adapter
  code on explicitly captured neutral and single-finger official local rotations;
  it must not be described as QMT's joint-axis estimation algorithm. QMT supplies
  the real quaternion projection for four-finger PIP and thumb IP constraints.
  Four-finger MCP retains spread/flexion while suppressing twist in a
  minimum-rotation functional basis; rotation of the remaining basis around the
  fitted axis is unobserved, not anatomically calibrated. Thumb MCP/CMC and the
  palm root are not adjusted. Uncalibrated joints retain the official path;
  missing DIPs follow the adjusted PIP only as labelled priors, never measurements.
  The unmodified official inference is still executed, but frames where this
  optional layer is actually applied are labelled
  `official_fsglove_with_qmt_functional_adapter_v1`, not numerically unchanged
  FSGlove output. Original official Euler/MANO results are retained alongside
  the adapted result; raw IMU data and official installation matrices are not
  overwritten. The adapter uses the upstream manotorch composer and preserves
  the official root rotation. It does not invoke `correct_local_inst_error`
  without real labelled MANO poses or fabricate labels for GPR. Neither axis
  fitting nor projection residuals establish anatomical accuracy, corrected
  heading drift, repaired dynamic sensor slippage, or full DiffHCal reproduction.
- PyTorch, manotorch, chumpy, OpenCV, deprecation, six and Polyscope are optional
  external dependencies, not bundled in this tool's source. The tested direct
  versions are recorded in [requirements-mano-tested.txt](requirements-mano-tested.txt);
  this is not a complete transitive dependency lock. Inspect the licenses of the
  exact packages shipped with an application before distribution.
- The optional `upstream` extra additionally installs Pydantic 2, scikit-learn,
  and tqdm. Their code and transitive dependencies retain their own licenses;
  they are external dependencies rather than source vendored by this adapter.
- The optional live GUI additionally installs PyQt5, Qt, pyqtgraph, PyOpenGL
  and pyserial; tested direct versions are in
  [requirements-live-tested.txt](requirements-live-tested.txt). In particular,
  [PyQt is GPL v3 / Riverbank Commercial License, not LGPL](https://www.riverbankcomputing.com/software/pyqt/).
  Qt has its own terms. Installing this prototype does not establish permission
  for a proprietary application distribution; preserve and review the relevant
  dependency notices before shipping a product. No Qt/PyQt sources are vendored.
- The tested manotorch source is pinned to
  [`a2a70c591f91551078b7bb2af9b5d9f275b626e0`](https://github.com/lixiny/manotorch/tree/a2a70c591f91551078b7bb2af9b5d9f275b626e0).
  Its top-level [LICENSE](https://github.com/lixiny/manotorch/blob/a2a70c591f91551078b7bb2af9b5d9f275b626e0/LICENSE)
  is Apache-2.0; the included `mano/webuser` files carry separate MANO research
  terms. A top-level code license does not override those file notices or grant
  rights to MANO model assets. Preserve the original notices.
- The tested chumpy source is the official upstream commit
  [`580566eafc9ac68b2614b64d6f7aaa84eebb70da`](https://github.com/mattloper/chumpy/tree/580566eafc9ac68b2614b64d6f7aaa84eebb70da),
  reporting version `0.71`, under its own MIT license. It includes the Python
  3.12 and NumPy import compatibility fixes; no upstream source or global
  NumPy/inspect monkey-patch is embedded in this adapter.
- Users may obtain MANO assets through the official site or a third-party
  source they trust and are entitled to use. A third-party URL does not replace
  the [applicable MANO terms](https://mano.is.tue.mpg.de/license.html), establish
  a distributor's authorization, or grant commercial-use/redistribution rights.
  Confirm that the intended use is permitted. Do not commit model pickle files,
  derived restricted assets or license acceptance credentials.

## Local model provenance (2026-09-30)

With the user's explicit approval, the integration environment obtained a right
hand model from the public
[Microsoft VITRA Hugging Face file page](https://huggingface.co/spaces/microsoft/VITRA/blob/main/weights/mano/MANO_RIGHT.pkl).
This records the actual source, not an endorsement or a separate license grant.

- Local path: `artifacts/glove_hand_pose_live/upstream_baselines_20260930/mano-assets/models/MANO_RIGHT.pkl`.
- Size: `3821356` bytes.
- SHA-256: `45d60aa3b27ef9107a7afd4e00808f307fd91111e1cfa35afd5c4a62de264767`.
- Isolated environment: `artifacts/glove_hand_pose_live/upstream_baselines_20260930/fsglove12-venv`.

The model remains a local artifact, not part of this tool's distributed source.
Matching the hash verifies file identity only, not pickle safety or legal rights.
Pickle deserialization can execute code; only load files from a trusted source.
The adapter does not automatically download assets or accept license terms.

Real MANO model and synthetic-pose solver round-trip tests have passed in that
environment. Three explicitly configured model poses were also rendered using
Polyscope's headless EGL backend and the resulting screenshots were inspected.
These checks do not establish physical glove accuracy, corrected heading drift,
personalized hand shape, or interactive desktop player acceptance. Model-only
screenshots must not be presented as reconstructions of measured IMU gestures.

## Official-core verification boundary (2026-10-01)

The external-source port has been compared with execution of the entire
original official calibration module: 64 synthetic frames across four output
mode combinations had zero maximum numerical difference. A 500-iteration
official MANO shape-optimizer smoke test reduced synthetic loss from 0.032624
to 0.006318. These checks demonstrate numerical parity and an executable
differentiable optimization path, not real sensor accuracy, verified contact
labels, individualized hand-shape accuracy, or corrected heading drift.

The v2 sparse-prior tests and known-axis VQF/MANO regressions use synthetic
ground truth only. Missing-DIP coupling does not establish personalized finger
kinematics or accurate physical fingertip contact. Calibration guidance now
shows approximately 50-degree upward/downward turns around the same palm axis,
following the pinned upstream README's recommendation of at least 90 degrees
between the two held poses, within the operator's comfortable range. The new
`calibration_motion_diagnostics` and non-blocking UI warnings assess capture
consistency only; they are adapter features, not upstream accuracy metrics,
functional-axis correction, or evidence that sideways motion or drift is fixed.

The separately enabled QMT functional adapter is an additional model-coordinate
constraint, not part of the zero-difference official-core comparison above.
Its per-joint capture diagnostics and interleaved held-out samples remain within
the same declared motion capture, not independent anatomical ground truth.
Saving a functional profile or obtaining smaller projection residuals does not
establish successful physical glove validation. No live-session restart or new
hardware acceptance is implied by installing or documenting this feature.

Six-axis fusion does not supply an absolute common heading, and this port does
not add online relative-yaw correction. Session/epoch bindings and explicit
inference labels remain necessary. Model screenshots, synthetic optimizer
losses, and successful source checks must not be presented as measured glove
performance.

Reference names do not imply endorsement, equivalent accuracy, or completed
paper reproduction. Legacy sparse calibration and constrained fitting remain
new experimental adapter code. Official numerical functions retain their
external upstream origin; the surrounding sparse completion, recording, UI,
and integration logic are not upstream-validated algorithms.
