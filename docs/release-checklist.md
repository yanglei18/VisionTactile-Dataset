# Release checklist

This checklist is for maintainers preparing a public source release. It is
separate from runtime acceptance: a repository can pass CI while untested on
hardware, and a hardware run can succeed while the repository still contains
private or generated artifacts.

For this multi-sensor workbench development tree, start with the
[source-only release preparation](../tools/release/README.md). It checks tracked
and new public source files without staging them, and can export a clean archive
without the existing Git history. A passing archive check does not authorize
pushing branches that still contain old captures or authentication screenshots.

## 1. Define the release

- Select one reviewed commit and an intended semantic version. Until the
  release is published, keep its changes under `Unreleased`; component package
  versions alone are not evidence that a matching project tag exists.
- Record supported Ubuntu, ROS 2, RealSense ROS, librealsense, firmware, and
  external PyVUT revisions.
- List user-visible changes and known limitations in `CHANGELOG.md`.
- Record the component package versions associated with the intended Git tag;
  independently versioned tools need not share one version number.
  The workbench is `0.6.0`; the standalone bilingual tactile viewer is `0.5.1`,
  declared in `tools/tactile_viewer_windows/product.toml`.
- Verify built Python wheel dependencies and ROS manifest version bounds. The
  manager, console and alignment replay extra require capture core >=0.6.0;
  partial upgrades must reject the previous 0.5.0 core. Run the deployment
  package-dependency tests as well as the usual full-install smoke checks.
- Review `THIRD_PARTY_NOTICES.md` and the license status of every pinned
  submodule and runtime dependency.
- Confirm which camera, Tracker, single/dual-hand pose and tactile modules are
  covered. Review the selected recording profile's exact Topic allowlist;
  enabling a preview or device never implicitly adds its topics to a bag.
- State what can be reproduced using public dependencies alone. The demo and
  software-only tests must not imply that the extended Tracker runtime,
  FSGlove source, MANO models or rig calibration are supplied.

## 2. Sanitize the public tree

The public repository must contain source, configuration examples, tests, and
documentation only. It must not contain raw evidence, captures, credentials,
role maps, generated bundles, camera scenes, logs, build output, or internal
design notes.

Review tracked paths before publishing:

```bash
git status --short
git ls-files | sort
git ls-files | rg '\.(bag|db3|mcap|pcap|pcapng|webm|zip)$'
git ls-files | rg '(^|/)(build|install|log|artifacts|bags|\.worktrees)/'
git submodule status
```

If sensitive material has ever been pushed, removing it in a later commit does
not remove it from Git history. Rotate exposed credentials if applicable and
use an explicitly reviewed history-rewrite or a new clean public repository.
Do not perform that operation as part of a routine release command.

## 3. Verify documentation

- The English and Chinese READMEs describe the same product scope.
- The [end-user manual](user-manual.md) can be followed from a fresh clone.
- The [single-entry calibration operations manual](../tools/tracker_camera_calibration/README.md)
  covers both embedded guided and standalone offline calibration, installation,
  input gates, recording, solving, three-run repeatability, versioned application,
  recovery, and acceptance. Implemented guidance and automated application must
  not be described as completed physical accuracy or endurance acceptance.
- Every public launch argument, command, topic, QoS profile, output path, and
  lifecycle state matches the implementation and
  [interface reference](interface-reference.md).
- Commands use repository-relative paths or documented environment variables;
  no developer worktree path remains.
- Limitations are explicit: no hardware exposure synchronization, no shipped
  device-specific camera/Tracker transform and no online extrinsic TF. The
  default v2 bag has 21 candidate topics, including six CameraInfo and three
  factory depth-to-color extrinsics. Camera–Tracker calibration is published on
  a separate typed topic and is not part of that default recording allowlist.
- Private prerequisites are labeled as such and are not implied to ship in the
  public repository.
- The storage documentation describes the START capacity budget and runtime
  low-space stop; it distinguishes those safeguards from sustained write
  throughput, received preview rates and independently verified bag quality.
- Package and archive contents include the applicable project license and
  third-party notices. A source release does not certify binary redistribution
  rights for bundled dependencies.
- The independent tactile viewer's bilingual Windows build instructions identify
  model and license inputs. Exclude local installers, MANO-derived static hand
  bundles and their build manifests from this source-only release; publishing
  a binary requires separate permission and target-platform acceptance.
- All relative Markdown links resolve and screenshots contain no private data.

## 4. Run software verification

From a non-Conda shell:

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --all --verbose
cd ..
# Run isolated pytest/Qt/Tk suites as documented in CONTRIBUTING.md.
python3 -m unittest discover -s tools/release/tests -v
python3 tools/release/prepare_release.py check
python3 tools/test_check_public_tree.py
python3 tools/check_public_tree.py
git diff --check
```

Do not waive failures in the public-tree checker. It is the release boundary,
not an informational lint.

Repeat installation from the prepared source in a fresh environment, without
editable dependencies pointing back into the development checkout. Build each
distributed Python wheel/source distribution and the ROS installation, inspect
their license and runtime resource files, and smoke-test installed commands.
Record the exact source revision, dependency versions and skipped checks; a
local run does not establish the result of remote CI.

## 5. Run reference hardware acceptance

Preserve sanitized textual results outside Git and record the exact tested
commit.

| Gate | Minimum evidence |
| --- | --- |
| Unified stream discovery | For `unified-dataset-v2`, verify the connected devices' coverage among the 21 candidate topics against the frozen required/optional plan. Absent optional devices do not block recording; legacy `unified-dataset-v1` bags retain their original 15-topic candidate contract. |
| Camera recording | START reaches `RECORDING`; planned or manual STOP reaches `COMPLETE`; `ros2 bag info` shows the exact topic allowlist. |
| Storage capacity | START accepts a sufficiently provisioned target and refuses an insufficient one; runtime low capacity or query failure triggers stop/finalization with a retained reason. Exercise failures with an isolated fake recorder/filesystem query, without filling a production disk. |
| Storage throughput | Measure the target filesystem under the intended workload and review recorded frames/gaps. The capacity budget and successful Recorder lifecycle do not establish write throughput or complete recording. |
| Tracker startup | Approved manual bootstrap completes and the read-only publisher owns the Dongle alone. |
| Tracker ROS output | `vt-vive-validate-topics` reports `status=PASS roles=3 identity_swaps=0 dropped=0`. |
| Tracker-camera calibration | Each final rigid pair exports `status: VALID`, holdout closure is at most 10 mm and 1 degree, and three independent runs meet the repeatability requirement. |
| Guided calibration | Fixed-board acquisition, movement guidance, three independent rounds, validation rejection, recovery and versioned application work on the final rigid rig. Verify the calibration topic's device identity, revision and validity expiry; collect physical accuracy and endurance evidence. |
| Unified offline alignment | `inspect`, `align`, and `validate` succeed; verdict is `ACCEPTED`; camera and Tracker coverage meet configured thresholds; all clock audits pass. |
| Visualization | Each role shows fresh data and follows physical motion; closing its GUI preview or RViz does not affect the publisher. |
| Workbench | Device configuration, manual Tracker initialization, per-card enlargement, record/start/stop, and output review work without terminal interaction after deployment. Preview enlargement does not change subscriptions or recording. |
| Optional hand modules | Independently verify each enabled hand's IMU/pose and 20×20 tactile stream, identity, recording allowlist, and offline alignment; right-hand results do not establish left-hand acceptance. |
| Independent Windows tactile viewer | In a clean target Windows installation, check offline installation, upgrade/uninstall, Chinese/English switching and saved preference, real COM connection/reconnection and native OpenGL rendering. Source tests and Wine checks do not establish Windows hardware acceptance. |
| Long run | Project-specific 300-second camera and Tracker evidence is reviewed for drops, gaps, reconnects, and thermal/storage behavior. |

`COMPLETE` is not a camera-data quality result. Hardware acceptance therefore
includes both Recorder lifecycle evidence and an independent data review.
For a source-only preview, publish the software evidence and explicitly mark
unperformed hardware gates as pending; do not describe them as passing.

## 6. Publish and verify

- Push only the reviewed public history and an intended signed or annotated
  version tag. For a first public snapshot, create the reviewed initial commit
  in a separate clean repository; do not push the development history implicitly.
- Create release notes containing compatibility, migrations, limitations, and
  links to the manual and changelog.
- Clone the public URL into a new directory with submodules and repeat the
  repository contract plus a source build.
- Verify issue and private security-reporting entry points on the destination
  repository. `SECURITY.md` does not enable GitHub private reporting; configure
  and verify it before advertising that route as available.
- Keep the previous supported tag and its rollback instructions available.

## 7. Support handoff

A useful issue report contains the software version/commit, OS and ROS
versions, hardware/firmware inventory, USB topology, command used, relevant
timestamps, expected versus actual behavior, and sanitized logs. Never request
or attach a raw camera scene, `pcapng`, private bundle, or role map in a public
issue.
