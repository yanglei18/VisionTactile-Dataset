# Hardware reference

This is the hardware configuration used while developing the public capture
workflow. It is a reproducibility reference, not a guarantee that every device
of the same model or every USB topology will behave identically.

| Camera | Model | Serial number | Firmware | ASIC serial |
| --- | --- | --- | --- | --- |
| `d405_1` | D405 | `260322278433` | `5.15.1.55` | `255323071625` |
| `d405_2` | D405 | `260322276463` | `5.15.1.55` | `255323071742` |
| `d436` | D436 | `408322071716` | `5.17.0.213` | `343123151280` |

In plain-text inventory form:

```text
d405_1 | D405 | 260322278433 | 5.15.1.55 | 255323071625
d405_2 | D405 | 260322276463 | 5.15.1.55 | 255323071742
d436   | D436 | 408322071716 | 5.17.0.213 | 343123151280
```

## USB topology

Three 1280x720x30 cameras place sustained load on the host USB topology. Connect
the D436 to an independent root bus from the two D405 cameras, then verify the
actual tree:

```bash
lsusb -t
```

Follow the path from each camera to its root hub/controller and confirm each
camera negotiated a `5000M` USB 3.x link. Separate physical sockets do not
prove separate controllers: motherboard ports can converge on one root hub, and
a front-panel pair may share the same upstream link. Move the D436 and run
`lsusb -t` again until the tree shows the intended separation.

`rs-enumerate-devices -s` is useful for mapping the configured serial numbers
to attached devices:

```bash
rs-enumerate-devices -s
```

USB paths can change after reconnecting or rebooting. Serial number is the
launch-time device binding; the USB tree is an operator diagnostic, not device
identity and not a Recorder completion condition.

## Stream-source facts

- The D405 color source is `depth_module`; both D405 launch instances
  configure `depth_module.color_profile` and
  `depth_module.color_format`.
- The D436 color source is `rgb_camera`; its launch instance configures
  `rgb_camera.color_profile` and `rgb_camera.color_format`.
- All cameras publish RGB8 color and Z16 depth at 1280x720x30. Infrared,
  depth-to-color alignment, RGBD composition, colorizer, and point-cloud output
  are disabled.

The live RealSense nodes may publish camera calibration, TF, metadata, and
extrinsics. The default Recorder allowlist contains only the six image topics
and three `CameraFrameTiming` topics, so those supporting interfaces are not
in the bag.

## Data and interpretation boundaries

Captured images, bags, MCAP files, and local hardware diagnostics belong in an
external data path such as `${HOME}/visiontactile-data`, never in Git.

The inventory documents the devices and settings used during development. It
does not establish cross-camera hardware exposure synchronization, spatial
calibration, absence of dropped frames, or compatibility with an untested
replacement camera. Operators must assess recorded data for their own
experiment independently of the Recorder lifecycle state.
