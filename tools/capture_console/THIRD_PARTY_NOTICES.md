# Capture console third party boundaries

The accompanying Apache-2.0 license covers independently authored workbench
code only. It does not relicense dependencies, external model files, hardware
software, or vendor trademarks.

The workbench installs PySide6 / Qt 6, pyqtgraph, PyOpenGL, and the local
`vt-capture-core`, `vt-tactile-capture`, and `vt-fsglove12` packages separately.
The glove calibration runtime uses a separate PyQt5 environment. Consult the
license metadata and notices of the exact dependency versions included in any
binary distribution; this package is a source distribution, not a grant to
redistribute every external component.

MANO files, external FSGlove source, the extended PyVUT runtime, and private
Tracker inputs are not included. Installing the workbench does not obtain
permission for or download these resources. Its 3D hand preview reuses a
separately deployed model and renderer; without that optional resource the
workbench retains its other views and diagnostics.
