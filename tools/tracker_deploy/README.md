# Tracker deployment tools

Build, verify, and install a private `.vttracker` package for an existing device set on Ubuntu 24.04, Linux x86_64, and Python 3.12. The public tool contains no private Tracker inputs. A package supplies approved extended PyVUT source, offline wheels, a matching bundle and role map, and the approved initialization capture.

From the repository root:

Use this tool from the complete trusted project checkout, normally through the
workstation's editable installation. Runtime validation reuses the repository's
Tracker identity and role policy; a standalone wheel is not a complete deployment
environment. The independent Windows tactile viewer does not need this package.

```bash
python3 tools/tracker_deploy/manage.py build --help
python3 tools/tracker_deploy/manage.py inspect --package "/absolute/path/device-set.vttracker"
python3 tools/tracker_deploy/manage.py install --package "/absolute/path/device-set.vttracker"
```

Use the [English deployment guide](../../docs/tracker-deployment.md) or [中文部署指南](../../docs/tracker-deployment.zh-CN.md) for prerequisites, building from a working workstation, installation with the desktop installer, and GUI import. Installation validates software and inputs without opening hardware or rewriting active workstation settings. Pairing, physical role assignment, and device acceptance remain separate.

Packages contain private device data and executable software. Transfer them privately between trusted deployment maintainers and the intended operators; do not add them to this public repository.
