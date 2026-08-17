# Contributing

Thank you for helping keep the public capture contract small, testable, and
truthfully documented.

## Development environment

- Develop and test on Ubuntu 24.04 with ROS 2 Jazzy.
- Use the system Python environment; Conda must not be active in capture, build,
  or test shells.
- Use RealSense ROS 4.58.1 when a change depends on the production wrapper.
- Use the Jazzy `ros2bag` CLI for runtime recording.
- Work on a focused feature branch. Do not develop directly on a release
  branch.

## Change workflow

Write or update a failing test before every behavior change, confirm that it
fails for the intended reason, then implement the smallest change that passes.
Documentation-only corrections must still satisfy the public-tree contract.

Before opening a pull request:

1. Build and run the software tests:

   ```bash
   test -z "${CONDA_PREFIX:-}"
   source /opt/ros/jazzy/setup.bash
   cd ros2_ws
   colcon build --event-handlers console_direct+
   source install/setup.bash
   colcon test --event-handlers console_direct+
   colcon test-result --all --verbose
   ```

   Do not use a symlinked install for this workflow.

2. Stage the intended public files, then run the repository checks. The
   public-tree checker reads Git index blobs deliberately, so an unstaged edit
   cannot make an older staged contract appear current.

   ```bash
   git add -- path/to/changed-file
   python3 tools/test_check_public_tree.py
   python3 tools/check_public_tree.py
   git diff --cached --check
   ```

3. Update English and Chinese user documentation together whenever public
   behavior, commands, compatibility, or limits change.

4. Describe lifecycle states exactly as the software implements them. In
   particular, `COMPLETE` means Recorder process termination was confirmed;
   do not present it as proof of message coverage, timing quality, absence of
   drops, or experiment suitability.

Keep the change focused and explain its observable effect and software-test
results in the pull request. Hardware observations can be useful context, but
they are not required acceptance gates for the Recorder-only lifecycle.

## Data and evidence policy

Do not commit captured data or local evidence, including bags, MCAP files,
images, videos, recordings, credentials, private scene data, device dumps, or
large generated logs. Keep those artifacts outside the repository.

If a change makes a hardware-specific claim, state the tested device and
topology narrowly and separate the observation from software guarantees. Share
only sanitized textual summaries through an appropriate review channel; never
add raw evidence to Git.
