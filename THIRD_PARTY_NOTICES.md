# Third-party notices

The Apache-2.0 license in this repository applies to this project's own files.
It does not relicense third-party software, firmware, hardware protocols,
drivers, or trademarks.

The prepared source archive contains this project's source and dependency
references, not a complete third-party runtime. It excludes PyVUT source,
external FSGlove source, MANO models, installed dependency binaries and private
device inputs. Source-candidate checks do not establish permission to obtain,
use or redistribute those external components, or full hardware reproducibility.

## PyVUT

The `pyvut` path is a Git submodule pointing to
`https://github.com/nijkah/pyvut` at the revision recorded by this repository.
At the pinned revision used for validation, the upstream checkout does not
contain a license file. The submodule's source is not covered by this project's
Apache-2.0 license, and this project does not grant redistribution rights for
it. Users and redistributors must review the upstream repository and obtain
any permissions they require.

The production Tracker Publisher also imports extended PyVUT modules that are
not present in the pinned public upstream revision. That approved checkout and
the private bootstrap inputs are separate prerequisites; they are not
distributed by this repository.

## ROS 2 and RealSense

ROS 2, the ROS packages installed from the operating-system package manager,
Intel RealSense SDK/wrapper components, and their transitive dependencies are
third-party works under their respective upstream licenses. They are runtime
dependencies and are not relicensed by this project. Consult the installed
package metadata and upstream source repositories before redistribution.

## Offline calibration dependencies

The optional offline Tracker-camera calibration tool uses NumPy, OpenCV,
PyYAML, and Pillow under their respective upstream licenses. They are declared
runtime dependencies and are not copied into or relicensed by this repository.
Redistributors must retain the notices required by the exact dependency
versions they package.

## Standalone glove hand-pose dependencies

The experimental tool in `tools/glove_hand_pose/` uses VQF, NumPy, pySerial,
and PyQt5 as runtime dependencies. In particular, PyQt5 has GPL v3/commercial
licensing choices; this repository's license does not replace those terms.
See the tool's [dependency and distribution notices](tools/glove_hand_pose/THIRD_PARTY_NOTICES.md)
before packaging it with its dependencies.

## FSGlove numerical-core integration and experimental sparse adapter

`tools/fsglove12/` can execute the original numerical definitions from a separately
obtained, hash-pinned FSGlove checkout (`upstream-live` / `upstream-shape`). Only
dependency wiring, hardware adaptation and the explicitly labelled sparse-input
completion are local code; the older self-written `live` solver remains a separate
legacy mode. No FSGlove source or MANO assets are bundled, and this is not an
official DiffHCal release or a claim to reproduce unpublished paper components.
Use of external source and models remains subject to their respective terms. See its
[dependency and model notices](tools/fsglove12/THIRD_PARTY_NOTICES.md).

## Capture workbench and legacy diagnostics

`tools/capture_console/` uses separately installed PySide6 / Qt 6, pyqtgraph,
and PyOpenGL. The glove windows use a separate PyQt5 environment; Qt bindings
are not interchangeable and their dependency licenses are not replaced by
this repository's Apache-2.0 license. The source archive does not contain Qt
binaries, Python environments, MANO models, or externally obtained FSGlove
source. Review the exact installed dependencies' license notices when building
a binary distribution; this source release is not a binary redistribution.

The root `Glove.py` and `Ploy_IMU.py` are the project's maintained legacy
STM32 diagnostic entry points supplied for this project. They are retained for
protocol parity and troubleshooting; their external Python dependencies keep
their own licenses. They are not advertised as upstream FSGlove source.

`Window_reference/` is a local reference checkout, not an install dependency
of the Linux packages. It is excluded in its entirety from the prepared source
archive, including generated Windows binaries and internal notes. Exclusion
from the archive does not erase copies in the original Git history.

## Product names

VIVE, VIVE Ultimate Tracker, VIVE Hub, SteamVR, Windows, Intel, RealSense, and
other product names are property of their respective owners. References in
this project identify compatibility and tested workflows; they do not imply
vendor endorsement or official vendor support.
