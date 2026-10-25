# Keyboard integration over SSH

This implements the SSH-verifiable part of [issue #2](https://github.com/takumi-nishimura/dora-openarm-data-collection/issues/2).
The official Keyboard node produces poses, the existing IK produces joint
targets, and the existing MuJoCo node simulates both arms. Their implementations
are reused through `modules/leader-keyboard.yaml`.

## Environment

From the repository root:

```bash
git submodule update --init --recursive
uv sync --locked
source .venv/bin/activate
```

Node revisions are fixed by the submodule commits and package versions by
`uv.lock`. Use a working branch such as `feat/keyboard-ssh` for these changes.
The parent project uses editable path dependencies, matching its existing node
dependencies; Keyboard is not a uv workspace member.

## Automated test without a display

```bash
uv run --locked python scripts/test-keyboard-headless.py
```

This starts the common dataflow locally with `LEADER=keyboard ROBOT=mujoco`,
viewer and rendering disabled. It requires ports 8000 and 8080 to be free and
does not use a Dora coordinator. A Python WebRTC peer sends the same `keydown`
and `keyup` messages as the browser. The test records input poses, joint targets,
and simulated observations for these cases:

- Initial pose agrees with the demo scene's home keyframe, using existing FK.
- Both arms receive translation along all three axes and move in MuJoCo.
- Both arms receive rotation and rotate in MuJoCo.
- Both grippers receive changing targets and move in MuJoCo.
- Targets remain unchanged after all keys are released.
- Collection UI Quit ends the whole dataflow with a successful exit code.

Artifacts are retained in a unique `out/keyboard-smoke-*/` directory:

- `dataflow.log`: node startup, execution, and shutdown logs.
- `joints/dataset/`: the common recorder's joint actions and observations.
- `targets/dataset/`: input poses recorded by an additional instance of the
  official recorder.
- `results.json`: sample counts and target/observed motion ranges.

The test descriptor is derived from `dataflow.yaml`. Only recording locations,
arm-only metadata, and the additional pose recorder are changed. Existing
datasets are not used. Temporary descriptors are removed on exit. Failed runs
retain their logs and stop their own process group. The test is also run by CI.

## Manual control

On a machine with a graphical session:

```bash
source .venv/bin/activate
LEADER=keyboard ROBOT=mujoco scripts/dora-envsubst run dataflow.yaml --uv
```

For a control-only process started over SSH:

```bash
source .venv/bin/activate
MUJOCO_VIEWER=false MUJOCO_RENDER=false \
  LEADER=keyboard ROBOT=mujoco scripts/dora-envsubst run dataflow.yaml --uv
```

Open `http://127.0.0.1:8080/`, focus the page, and use W/S, A/D, R/F for the
left arm and I/K, J/L, Y/H for the right arm. Hold Shift for rotation. C/X and
N/M operate the grippers. Use the collection UI at `http://127.0.0.1:8000/` to
Quit; from the SSH shell the equivalent is:

```bash
curl --fail --request POST http://127.0.0.1:8000/quit
```

To access both HTTP pages from another machine, forward their ports:

```bash
ssh -L 8000:127.0.0.1:8000 -L 8080:127.0.0.1:8080 <user>@<robot-host>
```

The management UI works over this HTTP tunnel. Keyboard controls use a WebRTC
data channel, whose media path also needs direct network reachability, a VPN,
or a suitable relay. An SSH TCP port forward alone does not tunnel WebRTC UDP.
The automated test runs its WebRTC peer on the server and works without that
remote-browser network path. Keyboard camera/video delivery is not wired in
this first integration.

## Recorded verification

On 2026-10-25, Ubuntu 24.04.5 over SSH with no DISPLAY or WAYLAND_DISPLAY:

- Python 3.12.3, Dora CLI/Python API 1.0.1, lock generation with uv 0.12.15.
- Base parent commit: `e398fa5656a79c76d12eb123ee3f16f54bff19ac`.
- Keyboard submodule: `ce50b2a9c4e917b45818ce9ada75abdb29b0d47c` (0.3.0).
- `uv sync --locked` succeeded before and after adding Keyboard. Existing
  dependency versions and existing submodule revisions were retained.
- The automated WebRTC -> IK -> MuJoCo -> recorder -> UI Quit test passed.
- The existing Keyboard integrator tests passed: 17 tests. On this ROS-enabled
  shell, run them with plugin auto-loading disabled to avoid unrelated ROS
  pytest plugins:

  ```bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --locked --with pytest==9.1.1 \
    python -m pytest nodes/dora-openarm-keyboard/tests/test_teleop.py
  ```

- The existing dummy/dummy configuration recorded an episode and exited through
  UI Quit successfully in an isolated recording directory.
- Ruff 0.14.11 checks passed for the headless test script.

The scene remains `demo` for this first wiring check. The Keyboard home pose
matches its model's home. Actual-robot geometry and mounting alignment is
[issue #3](https://github.com/takumi-nishimura/dora-openarm-data-collection/issues/3).
Keyboard publishes at its own nominal 500 Hz; IK uses `--tick-hz 500` to describe
that source, without adding rate conversion or changing the producer. Actual
rate and velocity tuning remains
[issue #7](https://github.com/takumi-nishimura/dora-openarm-data-collection/issues/7).

Manual browser focus loss, HOME return, remote-network WebRTC connectivity, and
local graphical viewing remain to be checked. Esc is not routed to UI Quit;
whole-dataflow shutdown and HOME interruption are tracked by
[issue #6](https://github.com/takumi-nishimura/dora-openarm-data-collection/issues/6).
No real-arm command or Quest session is part of this verification.
