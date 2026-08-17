# VIVE Ultimate Tracker Linux Validation

This workflow runs only after the Windows mapping gate passes. It validates
direct USB first, then one wireless Tracker, then all three wireless Trackers.

## Pinned PyVUT submodule

Use the repository submodule pinned to PyVUT commit
`7da6b081ad4ebfa0a0f2f242cdecb9ffc47341f1` and an isolated virtual
environment. The submodule contains the licensed live bootstrap and pose
stream implementation required by the production Tracker workflow.

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
export VT_PYVUT_ROOT="${VT_REPO}/third_party/pyvut"
export VT_PYVUT_VENV="${HOME}/.local/share/visiontactile/pyvut-venv"
git -C "${VT_REPO}" submodule update --init --recursive
git -C "${VT_PYVUT_ROOT}" rev-parse HEAD
python3 -m venv "${VT_PYVUT_VENV}"
"${VT_PYVUT_VENV}/bin/pip" install -e "${VT_PYVUT_ROOT}"
"${VT_PYVUT_VENV}/bin/pip" install -e \
  "${VT_REPO}/tools/vut_validation"
export PATH="${VT_PYVUT_VENV}/bin:${PATH}"
mkdir -p "${VT_DATA_ROOT}/tracker-validation"
```

The final `rev-parse` command must print the pinned commit above. The
submodule's `LICENSE` and `NOTICE` files are part of the checked release.

## udev

```bash
sudo install -m 0644 \
  tools/vut_validation/config/70-vive-ultimate-tracker.rules \
  /etc/udev/rules.d/70-vive-ultimate-tracker.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reconnect the device after installing the rules. Do not run the validator with
`sudo`.

## Direct USB gate

Connect one mapped Tracker with a data-capable USB-C cable and disconnect the
Dongle.

```bash
vt-vut-validate preflight --mode TRACKER_USB
vt-vut-validate run --mode TRACKER_USB --duration 30 \
  --expected-trackers 1 \
  --output "${VT_DATA_ROOT}/tracker-validation/usb-30s.json"
vt-vut-validate run --mode TRACKER_USB --duration 300 \
  --expected-trackers 1 \
  --output "${VT_DATA_ROOT}/tracker-validation/usb-300s.json"
```

## One-Tracker Dongle gate

Disconnect direct USB, connect only the Dongle, and ensure no other PyVUT
process is running. One long-lived process owns the HID device for each gate.

```bash
vt-vut-validate preflight --mode DONGLE_USB
vt-vut-validate run --mode DONGLE_USB --duration 30 \
  --expected-trackers 1 \
  --output "${VT_DATA_ROOT}/tracker-validation/dongle-one-30s.json"
vt-vut-validate run --mode DONGLE_USB --duration 300 \
  --expected-trackers 1 \
  --output "${VT_DATA_ROOT}/tracker-validation/dongle-one-300s.json"
```

## Three-Tracker Dongle gate

Use the already validated power-on sequence for the current bundle. All three
may be powered and allowed to reach their ready/green state before the single
validator process starts. The validator does not pair devices or repair a map.
Do not start a second PyVUT process.

```bash
vt-vut-validate run --mode DONGLE_USB --duration 60 \
  --expected-trackers 3 \
  --output "${VT_DATA_ROOT}/tracker-validation/dongle-three-60s.json"
vt-vut-validate run --mode DONGLE_USB --duration 300 \
  --expected-trackers 3 \
  --output "${VT_DATA_ROOT}/tracker-validation/dongle-three-300s.json"
```

Create the real identity-to-role mapping only under
`${VT_DATA_ROOT}/tracker-validation/`; never commit it.

## Timing and acceptance

The backend uses nonblocking HID so shutdown remains bounded. Each callback is
timestamped with host monotonic and host realtime clocks. PyVUT `timestamp_ms`
is an upstream host receive timestamp, not proven device time.

Every expected Tracker must sustain full status `2`, at least 30 Hz for 300
seconds, maximum gap at most 100 ms, zero disconnects, and zero USB
re-enumerations. Save kernel logs and JSON reports outside Git.

Do not clear maps, clear pairing, update firmware, reset devices, or send
unknown commands after a failure. Return to the last passing gate.
