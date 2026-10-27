<!-- SPDX-License-Identifier: Apache-2.0 -->
<!-- Added in the takumi-nishimura fork; see provenance.md. -->

# Standard Pedestal model alignment

This records the SSH-verifiable part of [issue #3](https://github.com/takumi-nishimura/dora-openarm-data-collection/issues/3).
The operator confirmed a standard OpenArm Pedestal with no changes to the arm
mounts or grippers. This selects the existing `openarm-mujoco` Pedestal model;
physical dimensions, zero offsets and motor directions still need measurement
during the Keyboard hardware check in
[issue #13](https://github.com/takumi-nishimura/dora-openarm-data-collection/issues/13),
before WebXR.

## One model for input, IK and simulation

`profiles/pedestal.yaml` selects the installed `pedestal/pedestal.xml`, the
`home` keyframe, the `arm_origin` site, and the two existing end-effector sites.
`scripts/model-node` passes these settings to the official node entrypoints.
It derives Keyboard's home position/orientation using existing FK and then
executes the official Keyboard node. IK, simulation and keyboard integration
remain implemented by their existing repositories.

The Keyboard, WebXR and UDP-VR IK modules use the shared profile. Future
OptiTrack input must express its poses in the same `arm_origin` frame. The
pose convention is meters and `[px, py, pz, qw, qx, qy, qz, gripper_angle]`.
The end-effector sites are the origins of the corresponding `ee_base_link`
bodies. The rigid-body-to-control-point transform is established in issue #5.

`OPENARM_MODEL=pedestal` selects the Pedestal profile explicitly. With
`ROBOT=pedestal`, it is also the automatic default. Other configurations retain
`demo` as the default. Each profile can select a bundled `asset` or a
repository-relative `xml` for a future measured workspace. Use the profile to
change model, frames or home; per-node overrides are rejected to avoid mixing
different models in one dataflow.

## Commands

With dependencies installed and the environment activated, run the stock
Pedestal scene on a graphical machine:

```bash
OPENARM_MODEL=pedestal LEADER=keyboard ROBOT=mujoco \
  scripts/dora-envsubst run dataflow.yaml --uv
```

For SSH control-only operation:

```bash
OPENARM_MODEL=pedestal MUJOCO_VIEWER=false MUJOCO_RENDER=false \
  LEADER=keyboard ROBOT=mujoco scripts/dora-envsubst run dataflow.yaml --uv
```

The Pedestal profile disables camera rendering by default. The stock model has
wrist cameras but no head or ceiling camera, and images are added later. The
normal `metadata_mujoco.yaml` still describes the demo camera set; these commands
are for control verification. The automated test uses isolated arm-only
metadata for its recordings. Production recording metadata is finalized with
the selected cameras in issue #9.

Reproduce the numerical and full-dataflow checks:

```bash
uv run --locked python scripts/test-model-config.py
uv run --locked python scripts/test-keyboard-headless.py --model pedestal
uv run --locked python scripts/test-keyboard-headless.py --model demo
```

The headless test records the resolved configuration from Keyboard, IK and
MuJoCo under `out/keyboard-smoke-<profile>-*/model-config/`. It verifies that
all three use the same profile, XML, frames and home before checking motion.
Joint actions, observations, input poses, metrics and logs are retained
alongside those reports. Run these tests locally for both models. The added
CI test step was removed on 2026-10-26 at the operator's request.

Inspect the model and driver without opening CAN devices:

```bash
uv run --locked python scripts/inspect-model.py --model pedestal
# Optional image preview on this SSH host, using EGL:
uv run --locked python scripts/inspect-model.py --model pedestal --preview
```

The JSON report and optional `preview.png` are written under
`out/model-alignment/pedestal/`. `--preview` renders one image without a display;
it does not establish the interactive viewer/browser acceptance condition.

## Model correspondence

The installed library versions used for verification are `openarm-mujoco 2.3.0`,
`openarm-control 0.4.0`, `openarm-driver 0.5.1`, and `mujoco 3.13.0`.
Compared with the original configuration:

| Item | Original demo | Standard Pedestal |
|---|---|---|
| Model | Cell, lifter and demo objects | Static pedestal and two arms |
| `arm_origin` in world, m | `[0.185, 0, 1.34]` | `[0, 0, 0.698]` |
| Right base origin in world, m | `[0.185, -0.031, 1.34]` | `[0, -0.031, 0.698]` |
| Left base origin in world, m | `[0.185, 0.031, 1.34]` | `[0, 0.031, 0.698]` |
| `nq / nv` | `26 / 25` | `18 / 18` |
| Head/ceiling cameras | Present | Absent |

Both arm bases have the stock model orientation. The origin rotation is the
identity; +X is forward, +Y is left and +Z is up. The seven arm joints are sent
as J1 through J7 followed by the gripper, in radians. Driver/MuJoCo round-trip
mapping and the seven-joint limits agree with the installed `openarm_pedestal`
driver configuration.

The arm home is J4 = pi/2 and the other six arm joints at zero. In `arm_origin`,
the right hand is approximately `[0.216, -0.1535, -0.22]` m and the left hand is
`[0.216, 0.1535, -0.22]` m, both with RPY `[0, -90, 0]` degrees. FK derives
these positions/orientations from the selected model; they are not copied from
the demo defaults. At this home, both profiles have essentially identical
relative hand poses despite their different world heights.

The gripper uses a different startup convention and is recorded separately:

| Gripper item | Right | Left |
|---|---|---|
| Pedestal model home | `0` | `0` |
| Driver start | `-0.4` | `0.4` |
| Keyboard initial open target | about `-0.7854` | about `0.7854` |
| Model limit | `[-0.7854, 0]` | `[0, 0.7854]` |
| Driver limit | `[-1.0472, 0.4]` | `[-0.4, 1.0472]` |

The arm home matches the driver start; the gripper state changes when targets
begin arriving. Gripper startup, alignment triggering and physical endpoint
verification are handed to issues #13 and #6. No driver limits or calibration
offsets were changed to hide these differences.

## Verification and remaining checks

On 2026-10-26 over SSH, all ten model/driver configuration checks passed and the
Pedestal headless WebRTC -> Keyboard -> IK -> MuJoCo -> recording -> UI Quit
test passed for both arms. The stock scene was also rendered with EGL for an
offscreen preview. The demo regression passed using the same suite.

Physical correspondence currently rests on the user-confirmed standard
configuration and the published stock model. The following remain for the
first Keyboard hardware check in issue #13:

- Confirm floor/base reference, mounting position and orientation against the
  physical device, and inspect any calibration offsets.
- Compare physical joint readings and signs with the model and FK.
- Check the control-point/gripper convention and the initial gripper behavior.
- Measure nearby tables/supports and add simple obstacle shapes when needed.
  The current scene includes the stock floor and pedestal; no workspace
  dimensions were guessed.

On 2026-10-26, the operator also confirmed the local Pedestal viewer and manual
browser key input on commit `fc24f13ea492d36552a6c900e520d077a12fd68e`, with
`OPENARM_MODEL=pedestal LEADER=keyboard ROBOT=mujoco MUJOCO_VIEWER=true
MUJOCO_RENDER=false`. This completes the graphical acceptance condition for
issue #3. The next step is Keyboard hardware verification in issue #13, followed
by WebXR in issue #4. HOME/focus-loss/Esc behavior remains issue #6. No headset
or real-arm session was run here.
