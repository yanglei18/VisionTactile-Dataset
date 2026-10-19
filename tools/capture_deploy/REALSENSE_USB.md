# RealSense RSUSB access

`python3 tools/capture_deploy/configure_realsense_usb.py check` is read-only and
prints JSON for the current user's USB access, matching devices, installed rule,
and usbfs buffer budget. It never opens cameras.

After stopping camera processes through the normal manager, an administrator can
run `apply` through the machine's normal authentication mechanism. From the
repository root, for example:

```sh
pkexec /usr/bin/python3 "$(pwd -P)/tools/capture_deploy/configure_realsense_usb.py" apply
```

This installs only `/etc/udev/rules.d/70-visiontactile-realsense-usb.rules` for Intel
USB products `8086:0b5b` and `8086:1156`, with mode `0660`, group `plugdev`, and
active-seat `uaccess`. It reloads rules and triggers only matching USB devices.
It raises `usbfs_memory_mb` to **64 MiB for the current boot**, preserving any
already larger or unlimited budget; no boot/module
configuration is written. Existing unrelated rules are never overwritten.

The original rule, buffer setting, and connected device permissions/ACLs are
saved in root-only `/var/lib/visiontactile-realsense-usb.json`. Application failure
attempts automatic rollback. To undo a successful application, use the same
authenticated command with `rollback`. A failed rollback retains the backup;
external edits cause refusal rather than being overwritten. After a reboot,
rollback leaves that boot's memory setting alone. Neither command stops nodes,
opens cameras, nor grants access to other USB product IDs.
If a new matching camera was connected after the original backup, rollback
reports its unknown original permissions and retains the backup instead of
claiming complete restoration.

Run `check` again as the desktop user after applying. Stop all camera processes
before switching between V4L2 and RSUSB; changing permissions does not release
interfaces already owned by an existing camera process.

At 1280×720 YUYV/Z16, the SDK's two bulk requests per stream require at least
`3 cameras × 2 streams × 2 requests × (1280 × 720 × 2 + 1024)` = 22,130,688 bytes
(21.11 MiB), before USB request overhead. The 16 MiB usbfs default is insufficient
for this RSUSB configuration. This constraint does not diagnose V4L2 frame loss.
