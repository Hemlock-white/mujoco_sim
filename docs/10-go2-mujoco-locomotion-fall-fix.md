# Go2 MuJoCo Locomotion Fall Fix

Date: 2026-04-21

## Problem

The Go2 MuJoCo simulation could enter `pd_stand`, but after switching to
`LOCOMOTION` the robot stepped in place, fell over, and repeatedly printed:

```text
[Recovery Balance] UpsideDown (1)
[FSM LOCOMOTION] Unsafe locomotion: roll is 164.125 degrees (max 40.000)
```

The repeated `roll is 164.125 degrees` message means the safety checker was not
the real cause. It was reporting the final result: the base had already rotated
close to upside down. The useful question was why the controller drove a visually
standing robot into that attitude immediately after locomotion started.

## Important Code Path

Runtime flow for this bug:

```text
teleop_client.py
  -> UDP command: stand / move
RL_Environment/udp_reader.py
  -> is_standing / is_moving / control_mode
mujoco_sim.py
  -> pd_stand() or RobotRunnerFSM.run()
RobotRunnerFSM.run()
  -> DesiredStateCommand.updateCommand()
  -> LegController.updateData()
  -> StateEstimator.update()
  -> ControlFSM.runFSM()
  -> LegController.updateCommand()
MuJoCo data.ctrl
```

The safety messages come from:

```text
MPC_Controller/FSM_states/FSM_State_Locomotion.py
MPC_Controller/FSM_states/FSM_State_RecoveryStand.py
```

The state and torque bridge between MuJoCo and the controller is:

```text
mujoco_sim/mujoco_sim_utils.py
```

## Root Causes

### 1. The Simulation Loaded Go2, But The Controller Used A1

`mujoco_sim.py` loaded:

```python
model = mujoco.MjModel.from_xml_path("assets/go2/scene.xml")
```

but initialized:

```python
robotRunner.init(RobotType.A1)
```

That made the MPC use A1 body mass, inertia, hip location, link lengths, body
height, and body name assumptions while controlling the Go2 XML model. The
values are close enough that the robot may appear to stand for a moment, but
they are not the same robot model.

Fix:

```python
robotRunner.init(RobotType.GO2)
```

and add a real `GO2` entry in `MPC_Controller/common/Quadruped.py`.

### 2. MuJoCo Actuator/Sensor Order Did Not Match Controller Leg Order

The controller assumes leg order:

```text
FL, FR, RL, RR
```

This is visible in `Quadruped.getHipLocation()`:

```python
x: leg 0 or 1 -> front
y: leg 0 or 2 -> left
```

So:

```text
0 = FL
1 = FR
2 = RL
3 = RR
```

The Go2 XML actuator and sensor order is:

```text
FR, FL, RR, RL
```

from `assets/go2/go2.xml`:

```xml
<motor name="FR_hip" ... />
<motor name="FR_thigh" ... />
<motor name="FR_calf" ... />
<motor name="FL_hip" ... />
...
<motor name="RR_hip" ... />
...
<motor name="RL_hip" ... />
```

If the controller torque vector is written directly into `data.ctrl`, the left
front torque goes to the right front actuator, the right rear torque goes to the
left rear actuator, and so on. This is a direct explanation for the observed
behavior: a robot that is already standing starts stepping in place with swapped
support and swing actions, then rolls over.

Fix:

```python
MUJOCO_ACTUATOR_LEG_ORDER = (1, 0, 3, 2)

def to_mujoco_ctrl_order(controller_leg_values):
    values = np.asarray(controller_leg_values, dtype=DTYPE).reshape(4, 3)
    return values[list(MUJOCO_ACTUATOR_LEG_ORDER)].reshape(12)
```

and write:

```python
data.ctrl[:] = to_mujoco_ctrl_order(legTorques)
```

For state input, use MuJoCo `qpos/qvel` joint order instead of the XML sensor
order, because `qpos[7:19]` and `qvel[6:18]` follow the body tree joint order:

```text
FL, FR, RL, RR
```

Fix:

```python
Dof_state["pos"] = data.qpos[7:19]
Dof_state["vel"] = data.qvel[6:18]
```

### 3. The Simulation Did Not Reset To The XML Home Keyframe

The Go2 XML defines a stable `home` keyframe:

```xml
<key name="home"
     qpos="0 0 0.27 1 0 0 0 0 0.9 -1.8 ..."
     ctrl="0 0.9 -1.8 ..." />
```

Without explicitly applying the keyframe, MuJoCo starts from default zero joint
positions after `MjData(model)` construction. That is not the intended standing
configuration.

Fix:

```python
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)
```

### 4. The Controller Was Running At The Physics Rate

MuJoCo timestep was set to:

```python
dt = 0.002
```

but controller timestep is:

```python
Parameters.controller_dt = 0.01
```

The old loop called `RobotRunnerFSM.run()` every MuJoCo physics step, meaning
the 100 Hz FSM/MPC logic was effectively called at 500 Hz. The MPC object itself
was still configured for `0.01`, so its internal gait timing and the external
call rate disagreed.

Fix:

```python
controller_decimation = max(1, int(round(Parameters.controller_dt / dt)))

if count % controller_decimation == 0:
    legTorques = robotRunner.run(dof_states, body_states, commands)
```

The last computed torque is still applied every physics step, which is normal:

```text
MuJoCo physics: 500 Hz
FSM/MPC update: 100 Hz
zero-order hold torque between controller ticks
```

### 5. The Safety Check Had Parentheses Bugs

Old code:

```python
if fabs(seResult.rpy[0]>deg2rad(max_roll)):
```

This computes the boolean comparison first, then applies `fabs()` to `True` or
`False`. It should compare the absolute roll angle:

```python
if fabs(seResult.rpy[0]) > deg2rad(max_roll):
```

Same issue existed for leg lateral position:

```python
if fabs(p_leg[1]) > 0.18:
```

This was not the main reason the robot fell. It was still important because
safety checks must report true physical conditions, not boolean artifacts.

### 6. Body Velocity Should Come From A MuJoCo Object Velocity API

The previous bridge used:

```python
data.cvel[body_id]
```

For the controller bridge, the clearer and safer method is:

```python
body_vel = np.zeros(6, dtype=np.float64)
mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, body_vel, 0)
```

This makes the requested frame explicit and avoids relying on a MuJoCo internal
array whose convention is easy to misuse while debugging world/body velocity
issues.

## Files Changed

### `MPC_Controller/common/Quadruped.py`

Added:

```python
RobotType.GO2
```

with Go2-specific:

```text
abad link length: 0.0955
hip link length: 0.213
knee link length: 0.213
abad location: [0.1934, 0.0465, 0]
body name: base_link
body mass: 15.205
body inertia: [0.107027, 0.0980771, 0.0244531]
body height: 0.27
```

### `mujoco_sim/mujoco_sim_utils.py`

Changed the simulator-controller bridge:

```python
get_dof_state()
get_body_state()
pd_stand()
to_mujoco_ctrl_order()
```

Key point:

```text
controller input order: FL, FR, RL, RR
MuJoCo ctrl order:      FR, FL, RR, RL
```

### `mujoco_sim.py`

Changed startup and control loop:

```python
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)
robotRunner.init(RobotType.GO2)
controller_decimation = max(1, int(round(Parameters.controller_dt / dt)))
data.ctrl[:] = to_mujoco_ctrl_order(legTorques)
```

### `MPC_Controller/FSM_states/FSM_State_Locomotion.py`

Fixed safety comparisons:

```python
fabs(seResult.rpy[0]) > deg2rad(max_roll)
fabs(p_leg[1]) > 0.18
```

## Validation

Syntax check:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
source ~/scripts/use_rlmpc.sh
python -m py_compile \
  mujoco_sim.py \
  mujoco_sim/mujoco_sim_utils.py \
  MPC_Controller/common/Quadruped.py \
  MPC_Controller/FSM_states/FSM_State_Locomotion.py
```

Headless simulation test:

```text
stand PD for 1 second
switch to LOCOMOTION with zero velocity command
run for 10 seconds
```

Observed result:

```text
unsafe: None
final z: about 0.289 m
final roll: about -0.23 deg
final pitch: about 1.93 deg
max abs roll: about 3.57 deg
max abs pitch: about 3.24 deg
```

This is far below the locomotion safety limit:

```text
max roll: 40 deg
max pitch: 40 deg
```

The important validation result is not only that the error disappeared. The more
important result is that the state bridge, robot model parameters, and actuator
order now agree for a zero-command locomotion case.

## How To Reproduce The Manual Check

Terminal 1:

```bash
cd ~/mujoco_sim
source ~/scripts/use_rlmpc.sh
python mujoco_sim.py
```

Terminal 2:

```bash
cd ~/mujoco_sim
source ~/scripts/use_rlmpc.sh
python teleop_client.py
```

Recommended sequence:

```text
stand
wait 1-2 seconds
move
do not press W/A/S/D yet
confirm it can stay in place without falling
```

Then test small commands:

```text
W once
Z to brake
Q or E once
Z to brake
```

Avoid jumping directly to high velocity. `teleop_client.py` currently uses:

```python
INCREMENT = 0.5
MAX_VEL = 4.0
```

Those limits are aggressive for early controller bring-up.

## Debugging Lessons

### Treat Safety Errors As Symptoms First

`Unsafe locomotion: roll is 164 degrees` was correct as a final physical state,
but it was not the first bug. Disabling the safety check would only hide the
report and let the robot keep applying bad torques while upside down.

### Robot Model Identity Must Match The Loaded XML

If the simulator XML is Go2, the controller model should be Go2. A1, Go1, and
Go2 share a quadruped structure, but MPC depends on exact geometry and inertia.
Small differences are amplified through contact force optimization.

### Always Audit Ordering At Simulator Boundaries

Most controller code assumes a fixed 12-vector order. Simulator XML files often
choose a different actuator or sensor order. The bridge must define this
contract explicitly.

The safest pattern is:

```text
read simulator native state
convert to controller canonical order
run controller
convert controller output back to simulator native order
```

### Keep Physics Rate And Controller Rate Separate

Physics can run faster than control. That does not mean the controller should be
called every physics step. A stable bridge should make this explicit with a
decimation factor.

### Use Keyframes For Known Good Initial Conditions

If an XML has a `home` keyframe, use it. Otherwise, debugging controller logic
while starting from unintended zero joints wastes time and can produce misleading
failure modes.

## Remaining Risks

This fix validates standing and zero-command locomotion. It does not fully tune:

```text
high forward velocity
large yaw commands
stairs or rough terrain
long-duration drift
teleop velocity limits
MPC weights specifically optimized for Go2
```

The next likely improvement is to reduce teleop command aggressiveness during
bring-up:

```python
INCREMENT = 0.1 or 0.2
MAX_VEL = 0.8 to 1.5
```

Then tune Go2 MPC weights and gait parameters after basic stability is confirmed.

