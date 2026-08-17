# Third-party notices

The Apache-2.0 license in this repository applies to this project's own files.
It does not relicense third-party software, firmware, hardware protocols,
drivers, or trademarks.

## PyVUT

The production Tracker integration uses the `third_party/pyvut` Git submodule,
pinned to commit `7da6b081ad4ebfa0a0f2f242cdecb9ffc47341f1` from
`https://github.com/yanglei18/pyvut`. That maintained distribution includes
the extended live bootstrap, HID, stream, and pose-decoder modules required by
the ROS 2 publisher. It is distributed under its own Apache-2.0 `LICENSE` and
retains upstream attribution in its `NOTICE` file.

The PyVUT distribution incorporates work from `nijkah/pyvut` and
`shinyquagsire23/vive_ultimate_tracker_re`; copyright and attribution for
upstream contributions remain with their respective authors. Private packet
captures, generated bootstrap bundles, Wi-Fi credentials, and physical-role
maps are not part of either public repository.

## ROS 2 and RealSense

ROS 2, the ROS packages installed from the operating-system package manager,
Intel RealSense SDK/wrapper components, and their transitive dependencies are
third-party works under their respective upstream licenses. They are runtime
dependencies and are not relicensed by this project. Consult the installed
package metadata and upstream source repositories before redistribution.

## VIVE hardware and software

VIVE, VIVE Ultimate Tracker, VIVE Hub, SteamVR, Windows, Intel, RealSense, and
other product names are property of their respective owners. References in
this project identify compatibility and tested workflows; they do not imply
vendor endorsement or official vendor support.
