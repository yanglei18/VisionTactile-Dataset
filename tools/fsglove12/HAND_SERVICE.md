# Managed single-hand runtime

`vt-fsglove12` 0.1.3 adds a headless service around the same
`ProcessLiveController` used by the independent hand display. Each service owns
one spawned solver, one hand configuration and one control endpoint. Numerical
fitting, input fusion, functional calibration and shape optimization retain the
saved independent implementation. No Qt module is required by the service.

```sh
python -m vt_fsglove12 hand-service \
  --hand right --resources /absolute/runtime/right \
  --control-socket /absolute/private/right.sock \
  --output-root /absolute/recordings/right
```

The service prepares resources without opening hardware. Explicit `--port`
requests one startup connection; it does not reconnect automatically. The
alternative resource arguments are `--config`, `--rest`, `--mano-assets` and
`--fsglove-source`, supplied together without `--resources`. `--ros-socket`
retains the existing results-only ROS datagram publisher. `--output-rate-hz`,
`--record-streams`, `--replay-raw` and `--replay-speed` have the controller's
original meanings. Each hand needs a different socket and output directory.

The shared `hand_service_resources.validate_bundle_manifest(root, *, hand)`
returns `(resolved_root, manifest)` after checking declared paths, hand, SHA256
and the 2 MiB JSON limit. It imports no numerical or GUI dependencies. The UI
adds its own display/tactile semantic checks after this shared validation.

The ROS `hand_service.launch.py` exposes the same four explicit resource paths,
including `rest:=/absolute/rest.json`. An incomplete explicit selection fails
before creating a process; the launcher does not search for an adjacent rest
file or substitute another hand's resources. When a legacy profile also selects
`resources`, nonempty explicit fields must identify the corresponding members
of that same bundle. They are then normalized to one bundle argument for the
worker. Conflicting installations are rejected. Left-hand rest files must
explicitly identify the left hand, and the selected MANO directory must contain
the requested hand model.

## Local control contract

`hand_service_protocol.request(socket_path, document, *, timeout=1.0)` is a
standard-library-only client. Requests and responses are length-prefixed UTF-8
JSON with a four-byte network-order unsigned length. Version 1 limits requests
to 64 KiB and responses to 4 MiB, rejects duplicate keys and nonfinite JSON, and
performs no automatic retry. The socket is mode 0600, same-user connections
only on Linux, with four bounded handlers and a bounded receive deadline.

A snapshot request is `{"schema_version":1,"op":"snapshot","hand":"right"}`.
It returns `ok`, `service_instance_id`, `hand`, `service_state`, `pose_valid`,
`calibration_busy`, `calibration_activity`, `snapshot`, `receipts` and
`snapshot_monotonic_ns`. Optional `expected_service_instance_id` binds a read to
an already observed service. `snapshot` retains the independent controller's
entire payload, including the separate `input_snapshot` mailbox. Source sample
times, input quality, per-joint validity and display-only states remain intact.

`ready` means the initialized compute process is alive. It does not mean a
device is connected or a valid hand pose is available. `pose_valid` uses the
existing results-only export contract, including source timestamps, joint
validity and input health, and requires a connected source.

Every mutation supplies the original controller command plus exact bindings:

```json
{
  "schema_version": 1,
  "op": "command",
  "hand": "right",
  "expected_service_instance_id": "observed-instance-id",
  "request_id": "unique-action-id",
  "command": "capture",
  "args": {"step": "neutral"},
  "expected_session_id": "observed-session-id",
  "expected_profile_version": null
}
```

Session/profile bindings can be null before connection/calibration. They are
checked at service admission, worker admission and immediately before execution.
All original controller commands remain available. An identical request ID and
payload returns its existing receipt; reusing that ID for another action fails.
Rejected IDs are retained as well. The service holds at most 1024 IDs without
eviction; reaching that bound rejects additional commands. Snapshots expose the
last 64 receipts; any existing ID is queryable with `op:"receipt"`, `hand`,
`schema_version`, `expected_service_instance_id` and `request_id`.

Receipt states are `queued`, `executed`, `rejected` and `unknown`. `queued`
means accepted into a bounded queue; `executed` means the command body finished.
An asynchronous capture or optimization may still be running. Calibration and
storage outcomes remain in the original snapshot. A process exit before
confirmation produces `unknown`. A timeout never repeats a mutation, including
after an instance changes. Inspect its receipt before deciding on a new action.

`calibration_busy` is the recording exclusion boundary: it is true during
pending service commands, published capture/fitting/shape work, initialization,
or missing activity information. Both IPC paths carry a small
`runtime.calibration_activity` atomically paired with their command completion
counter. The service chooses the most recent activity observation, so a fast
input acknowledgement cannot expose an older mesh mailbox's idle state.
Synchronous shape application, save and restore are covered by the pending
receipt until completion. Storage errors and unsaved artifacts are distinct
from work still running and remain explicitly visible in the snapshot.

The workbench manager owns mutation policy, recording exclusion and lifecycle;
an embedded panel can poll snapshots directly in its background worker. Neither
GUI polling nor socket errors open a device, fit a profile or restart a service.
SIGTERM/SIGINT close the original reader/recorder cooperatively and remove the
owned socket only after the worker exits.

## Historical profile provenance

Packaged FSGlove sources are checked against pinned file SHA256 values and
unmodified numerical AST definitions. Their directory is not an independent
Git checkout. Older inspection accidentally recorded the enclosing application
repository's HEAD. The adapter now records no checkout HEAD for such packages.
A historical profile with that old hexadecimal HEAD remains usable only when
all other provenance is exactly equal, including paths, source hashes, AST
symbols, model root and numerical dependency identity. The historical profile
file is never rewritten. Independent FSGlove Git checkouts still require their
actual HEAD to match. Changed model/configuration, source bytes, dependency
paths or session/filter epochs are not silently accepted.

## Repeatable numerical comparison

`scripts/compare_hand_runtime.py` launches two isolated workers with explicitly
selected package sources and the same numerical Python. It never imports a
device source or edits a recording. Each case supplies a session containing
`config.json`, `rest.json`, exactly one `input_scale_*/raw.csv` and one original
capture for each of the three mounting poses under `calibration/`. Duplicate
captures must be selected into a separate comparison corpus before running;
the tool never guesses which historical attempt was accepted.

```sh
python3 tools/fsglove12/scripts/compare_hand_runtime.py \
  --baseline-source /baseline/tools/fsglove12/src \
  --candidate-source /candidate/tools/fsglove12/src \
  --python /numerical/venv/bin/python \
  --case right /records/right-session /resources/right/upstream /resources/right/mano \
  --case left /records/left-session /resources/left/upstream /resources/left/mano \
  --raw-frames 1000 --output /new/comparison-directory
```

The tool refits the saved captures separately in each implementation, then
compares every captured frame's numerical pose, anatomical angles, root,
quality/validity/observation masks and MANO joints. It also compares the complete
fusion output for the fixed raw prefix. JSON hashes must be exactly equal;
a mismatch or a source import outside the selected tree exits unsuccessfully.
Existing profiles are not rewritten. Worker inputs/logs retain local paths in
the chosen output directory; `comparison.json` contains only input content
identities and numerical results and is suitable for sharing.

[The recorded integration comparison](evidence/hand_runtime_equivalence_20261108.json)
contains both hands' capture/raw hashes, exact equality results, and the
separate unchanged historical-profile acceptance checks. Those checks prove
software equivalence for these records; they do not measure anatomical
accuracy or add observations absent from the twelve sensors.
