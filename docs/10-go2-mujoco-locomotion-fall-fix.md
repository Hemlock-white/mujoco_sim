# Go2 MuJoCo / SDK2 Locomotion Debug Note

Date: 2026-05-21

This note explains why the Go2 can stand but cannot yet locomote reliably in the
current MuJoCo MPC and SDK2 DDS experiments. It is written as a debugging guide:
first understand the coordinate/index contracts, then isolate the fault factor,
then apply a small correction.

## Current Test Matrix

The observed results are:

| Case | Setup | XML actuator | XML sensor | Result |
| --- | --- | --- | --- | --- |
| 1 | `go2_rl_test.py` + `mujoco_sim_sdk2.py` | either order | either order | Stand/sit both work (RL policy has explicit remap) |
| 2 | commit `f4d6a82`, `mujoco_mpc_locomotion.py` | FL-first | FL-first | Stand + all WSAD/QEZC **work normally** |
| 3 | commit `f4d6a82`, `mujoco_mpc_locomotion.py` | FL-first | FR-first | Stand **immediately explodes** (legs twist) |
| 4 | commit `fc5d751`, `mujoco_mpc_locomotion.py` | FR-first | FR-first | Stand works; `w` walks 1–3 steps then falls right |
| 5 | current `fixDDS`, SDK2 + teleop | FR-first | FR-first | Stand works; `w` dives right-front |
| 6 | current `fixDDS`, `RL_MPC_Locomotion.py` Aliengo + teleop | — | — | All WSAD/QEZC work normally |
| 7 | `fc5d751`, `mujoco_mpc_locomotion.py` | FL-first | FL-first | Stand + all WSAD/QEZC **work normally** (additional confirmation) |

The pattern across cases 2–5 reveals why **stand passes but locomotion fails**:

- `STAND_TARGET` hip angles are all **0.0** for all four legs. A symmetric target is
  insensitive to FL/FR swap — the same angle is applied regardless of which physical
  leg is addressed.
- Locomotion is **asymmetric**: trot requires diagonal pairs FL+RR vs FR+RL to
  alternate, abduction Jacobians depend on correct side sign, and MPC GRF is
  computed per leg with individual foot placement geometry.

Case 3 shows the worst failure mode: actuator order is correct (FL-first) but
sensor order is wrong (FR-first). State estimation feeds FR data as if it were FL.
MPC computes a torque from wrong state and sends it to the *correct* actuator — no
cancellation, immediate explosion.

Cases 4 and 5 show the double-mismatch pattern: both sensor and actuator are
FR-first. Wrong state and wrong actuator targeting partially cancel during symmetric
stand, but break during locomotion where asymmetric forces are required.

## The Controller's Leg Contract

`MPC_Controller/common/Quadruped.py` defines hip locations as:

```python
x: leg 0 or 1 -> front, leg 2 or 3 -> rear
y: leg 0 or 2 -> left,  leg 1 or 3 -> right
```

Therefore the MPC controller leg order is:

```text
MPC leg order: FL, FR, RL, RR
```

This same ordering is used by:

- `LegController.updateData()`
- `LegController.computeLegJacobianAndPosition()`
- `ConvexMPCLocomotion` gait phase arrays
- `getSideSign()`, where left legs are positive and right legs are negative

If the controller thinks leg 0 is FL but the input data for leg 0 is actually
FR, then the abduction Jacobian and lateral foot placement are wrong. The robot
may still stand, but a trot will push the body sideways or roll it over.

## The Go2 SDK2 / Unitree Contract

The Unitree Go2 low-level motor order is commonly:

```text
SDK2 motor order: FR, FL, RR, RL
```

The RL SDK2 example confirms this. In
`crazydog_mujoco/deploy_real/go2/go2_rl_test.py`, the policy does not consume
raw SDK2 joint arrays directly. It uses:

- `jointIDRemapping()` to convert SDK2 motor states into policy observation order
- `cmdRemapping()` to convert policy actions back into SDK2 motor command order

That is why the RL policy can work with SDK2: it has an explicit boundary map.

## Why FR-First XML Breaks MuJoCo MPC

When `go2.xml` actuator and sensor order is:

```text
FR, FL, RR, RL
```

but `mujoco_mpc_locomotion.py` does:

```python
Dof_state["pos"] = data.sensordata[0:12]
Dof_state["vel"] = data.sensordata[12:24]
data.ctrl[:] = legTorques
```

then the MPC vector is interpreted as if it were already `FL, FR, RL, RR`, but
MuJoCo is actually providing/applying `FR, FL, RR, RL`.

The physical symptom matches the tests:

1. `pd_stand` can work because the target joint angles are nearly the same for
   all four legs.
2. `move` can stand briefly because the first few stance forces may still be
   close enough.
3. After 1-3 steps, lateral errors accumulate. The controller may command a
   left-leg stabilizing force into a right leg, or swing/stance pairs are
   applied to the wrong side. The body falls right or right-front.

This is especially visible in trot because diagonal pairs should alternate:

```text
FL + RR  <->  FR + RL
```

If FL/FR or RL/RR are swapped at the boundary, the gait table may be correct
inside MPC but wrong at the robot.

## Why SDK2 DDS Locomotion Also Fails

The SDK2 path has the same semantic risk plus DDS timing:

```text
mujoco_sim_sdk2.py
  publishes LowState / SportModeState
mpc_locomotion_sdk2.py
  reads LowState / SportModeState
  computes MPC torque
  publishes LowCmd
UnitreeSdk2Bridge
  applies LowCmd into mj_data.ctrl
```

From the controller's perspective, SDK2 messages look like a real Go2. That is
good for deployment, but it also means the controller must respect the real Go2
motor order. If `get_dof_state_sdk2()` directly maps:

```python
Dof_state["pos"][i] = low_state.motor_state[i].q
```

then MPC receives SDK2 order, not MPC order. Likewise, if MPC torque is written
directly to `motor_cmd[i].tau`, torque goes out in MPC order, not SDK2 order.

The likely primary fault factor is therefore:

```text
missing SDK2/FR-first <-> MPC/FL-first remap
```

DDS latency and asynchronous `LowState`/`SportModeState` can amplify the fall,
but they are less likely to be the first cause because the failure direction is
consistent with leg-order mismatch, and because the RL SDK2 test works when it
uses explicit remapping.

## Root Cause Summary (Confirmed)

The tests above, combined with reading the official `unitreerobotics/unitree_mujoco`
and `unitreerobotics/unitree_sdk2_python` repositories, confirm the following:

| Layer | Order | Source |
| --- | --- | --- |
| Unitree Go2 real robot motor bus | FR, FL, RR, RL | SDK2 hardware spec |
| `unitree_mujoco` official `go2.xml` | FR, FL, RR, RL | official repo XML |
| Current `fixDDS` `go2.xml` | FR, FL, RR, RL | both actuator and sensor |
| MPC controller internal (`Quadruped.py`) | FL, FR, RL, RR | `getHipLocation()` y-sign |

The bridge (`unitree_sdk2py_bridge.py`) correctly mirrors the XML actuator/sensor
order into the SDK2 `LowState`/`LowCmd` messages. This is the intended design:
from the controller's perspective the simulator and the real robot look identical.

The missing piece was a **boundary remap** in the controller, translating
SDK2/FR-first ↔ MPC/FL-first at exactly two call sites:

1. Reading joint state into MPC (`get_dof_state`, `get_dof_state_sdk2`).
2. Writing MPC torque back to the robot/simulator (`data.ctrl`, `motor_cmd[j].tau`).

## Applied Fix

The fix is an explicit boundary remap in the controller layer. The bridge and XML
are left unchanged, preserving compatibility with the official Unitree tooling and
the real robot.

### The permutation

```python
# mujoco_sim/mujoco_sim_utils.py
LEG_SDK2_TO_MPC = np.array([3,4,5, 0,1,2, 9,10,11, 6,7,8], dtype=int)
```

Meaning: to convert a FR-first (SDK2) index array to a FL-first (MPC) index array,
reindex with this map. Because FL/FR and RL/RR are simply swapped, the permutation
is its own inverse — the same array is used for both read and write directions.

### State reading (both paths)

```python
# get_dof_state()  — direct MuJoCo MPC path
Dof_state["pos"] = data.sensordata[0:12][LEG_SDK2_TO_MPC]
Dof_state["vel"] = data.sensordata[12:24][LEG_SDK2_TO_MPC]

# get_dof_state_sdk2()  — SDK2/DDS path
j = LEG_SDK2_TO_MPC[i]
Dof_state["pos"][i] = low_state.motor_state[j].q
Dof_state["vel"][i] = low_state.motor_state[j].dq
```

### Torque writing (both paths)

```python
# mujoco_mpc_locomotion.py  — direct MuJoCo MPC path
data.ctrl[:] = legTorques[LEG_SDK2_TO_MPC]

# mpc_locomotion_sdk2.py  — SDK2/DDS path
j = LEG_SDK2_TO_MPC[i]          # MPC leg i → SDK2 motor j
self.low_cmd.motor_cmd[j].tau = legTorques[i]
```

`pd_stand` is **not** remapped because `STAND_TARGET` is fully symmetric (all hip
angles = 0.0), so the mapping has no effect on the stand result. If non-symmetric
stand targets are introduced later, `pd_stand_sdk2` will also need the remap.

The RL policy (`go2_rl_test.py`) used `jointIDRemapping` / `cmdRemapping` for the
same reason. The approach here follows the same principle: keep the controller
internal order stable, translate only at the robot/simulator boundary.

## Debug Logging And Plotting

Two debug CSV paths are useful.

For SDK2:

```bash
SDK2_DEBUG_LOG=1 SDK2_DEBUG_LOG_DIR=logs/sdk2_debug python mujoco_sim_sdk2.py
python mpc_locomotion_sdk2.py --debug-log --debug-log-dir logs/sdk2_debug
```

For direct MuJoCo MPC:

```bash
python mujoco_mpc_locomotion.py --debug-log --debug-log-dir logs/mujoco_mpc_debug
```

Plot either log directory:

```bash
python -m mujoco_sim.plot_locomotion_debug --log-dir logs/sdk2_debug
python -m mujoco_sim.plot_locomotion_debug --log-dir logs/mujoco_mpc_debug
```

Generated plots:

- `latency_timing.png`: DDS/loop timing
- `imu_state_estimator.png`: IMU and state estimator motion
- `foot_contact_forces.png`: measured contact forces from MuJoCo contacts
- `mpc_state_input.png`: MPC state `x`, desired `x_des`, and GRF input `u`
- `foot_actual_desired.png`: current and desired foot positions

## Success Criteria Before Real Robot Deployment

Before moving to a physical Go2, all of these should be true in MuJoCo:

1. `pd_stand` reaches a stable symmetric stance.
2. In `move` with zero velocity, trot can idle without drifting sideways.
3. One `w` command produces forward velocity without persistent right/left roll.
4. Measured foot contacts match MPC contact phase.
5. MPC predicted vertical GRF is applied to the same physical foot.
6. LowCmd publish period and bridge receive age are stable.
7. No index conversion is hidden inside random call sites; all robot-boundary
   conversion lives in one named helper/module.

## Current Working Hypothesis

The main fault is not that Go2 cannot be controlled by this MPC. The main fault
is that the Go2 SDK2/MuJoCo boundary does not currently preserve the MPC
controller's assumed leg order.

Fix the boundary map first. Then use the latency plots to decide whether DDS
thread timing needs additional locking or rate alignment.

---

## Deep Dive: Why the SDK2 Path Still Falls After the Boundary Fix

*This section is written for a reader who is new to robot control and hardware
communication — a third-year undergrad who has just started working on legged
robots. It explains two remaining failure modes from first principles, with
diagrams.*

### Background: How MPC Locomotion Works (One-Paragraph Primer)

The MPC controller runs a continuous feedback loop:

```
Sensors → State Estimator → MPC Optimizer → Leg Controller → Actuators
   ↑                                                               |
   └───────────────────── (physical robot) ───────────────────────┘
```

Every 5 ms the loop:
1. Reads joint angles/velocities and IMU orientation.
2. Feeds them to the **State Estimator**, which computes body position,
   velocity, and ground contact.
3. Passes the estimated state to the **MPC**, which solves a short-horizon
   optimization to find the best ground-reaction forces for the next ~0.2 s.
4. Converts those forces into joint torques and sends them to the motors.

If any of these steps receives *wrong or stale* information, the resulting
torques can be wrong — and for a robot balancing on four tiny feet, "wrong
torques" means falling.

---

### Problem 1: Cold-Start from `robotRunner.reset()`

#### What the State Estimator Actually Tracks

The `StateEstimator` is not a simple sensor reader. It is a running estimate
that accumulates history. The three most important pieces of memory it holds are:

```
┌─────────────────────────────────────────────────────────┐
│              StateEstimator internal memory              │
│                                                          │
│  _foot_contact_history  — which feet have been on the   │
│    ground recently; used to estimate body height and     │
│    ground slope.                                         │
│                                                          │
│  ground_R_body_frame    — the rotation that describes    │
│    how the ground plane is tilted relative to the body;  │
│    used to project gravity correctly.                    │
│                                                          │
│  _contactPhase          — which feet are currently in    │
│    stance vs swing; used to set MPC contact constraints. │
└─────────────────────────────────────────────────────────┘
```

These three variables are built up over many control cycles. If the robot has
been standing for 2 seconds, the estimator has accumulated ~400 samples of
contact information and has a very accurate picture of the ground.

#### What `reset()` Does

`RobotRunnerFSM.reset()` calls `ControlFSM.initialize()`, which calls:

- `StateEstimator.reset()` — sets all three variables above back to `None`
  or zero.
- `ConvexMPCLocomotion.initialize()` — resets `iterationCounter = 0` and
  `firstRun = True`, so the gait phase restarts from the beginning.
- `firstSwing = [True, True, True, True]` — every leg thinks it is starting
  its very first swing.

Visualized as a memory wipe:

```
BEFORE reset()                      AFTER reset()
──────────────                      ─────────────
contact_history: [████████████]     contact_history: None
ground_R_body:   [well-estimated]   ground_R_body:   None
_contactPhase:   [0.8 0.0 0.0 0.8] _contactPhase:   [0 0 0 0]
iterationCounter: 847               iterationCounter: 0
firstSwing: [F F F F]               firstSwing: [T T T T]
```

The robot has just spent 2 seconds standing still in a well-estimated state.
`reset()` erases all of that in one call.

#### What Happens in the First Few MPC Steps After reset()

The MPC optimizer needs the state estimate to compute ground reaction forces.
Here is what it receives immediately after reset, and why each item causes a
bad decision:

```
Cycle 0 (right after reset):
  _foot_contact_history = None
    → _init_contact_history() must be called first (happens inside cMPC.run(),
      on the first step). Until then, foot positions relative to ground are
      unknown.
    → MPC treats all feet as if they are at z = 0 (flat ground assumption).
      If the robot is actually at z = 0.27 m (standing), every foot position
      estimate is wrong by 0.27 m.

  ground_R_body_frame = None
    → The estimator falls back to the identity rotation (flat ground).
      If the robot is slightly tilted from stand, gravity projection is wrong.
    → MPC plans forces for a flat surface but the robot is not flat.

  _contactPhase = [0, 0, 0, 0]
    → The MPC thinks all four feet are in swing (no stance foot).
      It will not compute support forces; instead it will command swing
      trajectories for all legs simultaneously.
    → All four legs lift off at once → immediate collapse.
```

A timeline of estimator quality:

```
Time from reset()
│
│  0 ms  ───→  reset() called, all history erased
│  5 ms  ───→  cycle 0: _init_contact_history() runs, but only 1 sample
│              MPC result: low quality (treats ground as flat, contact unknown)
│ 10 ms  ───→  cycle 1: 2 samples of contact history
│              MPC result: still poor, firstSwing=True for all legs
│ 15 ms  ───→  cycle 2: contact history improving
│ 50 ms  ───→  ~10 cycles: estimator begins to converge
│100 ms  ───→  ~20 cycles: contact phase tracking reasonably good
│              By this time a 5 Hz trot has already completed half a cycle.
│              The robot has been receiving wrong torques for that entire time.
│
│  FALL WINDOW: 0 ms → ~50 ms after reset()
```

#### Why the Direct MuJoCo Path Does Not Have This Problem

In `mujoco_mpc_locomotion.py`, `robotRunner.reset()` is **never called**. The
runner is initialized once at startup and runs continuously through both stand
and move phases:

```
Direct MuJoCo path:
  t=0:      robotRunner.init()       ← one-time initialization
  t=0..2s:  pd_stand → leg torques  ← StateEstimator.update() called every  #its not
                                        5 ms during stand; estimator warms up
  t=2s:     user presses move        ← StateEstimator already has 400 samples
                                        of contact history; ground_R_body known
  t=2s+:    MPC locomotion           ← estimator is WARM → good torques
                                        from the very first step
```

The stand phase acts as a free warm-up for the state estimator. The first MPC
step after entering move mode uses accumulated history — not cold zeros.

#### Why `robotRunner.reset()` Was Added in the SDK2 Path

In `mpc_locomotion_sdk2.py`, the original code had:

```python
if not was_moving:
    robotRunner.reset()   # ← THIS IS THE PROBLEM
    was_moving = True
```

The intent was: "ensure a clean start when the user first enters move mode."
The reasoning sounds sensible — start fresh rather than using state that was
built during a different control mode. But in practice it has the opposite
effect: it discards exactly the information MPC needs.

#### Fix A: Remove the `robotRunner.reset()` Call

By not calling `reset()`, the state estimator retains all the history it built
during the stand phase. The transition from stand to move is seamless from the
estimator's perspective. The FSM handles the state transition internally
(RECOVERY_STAND → LOCOMOTION) without needing a full reinitializataion.

---

### Problem 2: DDS 5 ms Round-Trip Latency

#### The Communication Architecture

The SDK2 path uses three separate processes that communicate via DDS (a
publish-subscribe messaging system):

```
Process A: mujoco_sim_sdk2.py (MuJoCo Simulator + Bridge)
Process B: mpc_locomotion_sdk2.py (MPC Controller)

  Process A                              Process B
  ─────────                              ─────────
  mj_step()                              read LowState
  ↓                                      ↓
  sensordata → LowState ──DDS──→         get_dof_state_sdk2()
                                         get_body_state_sdk2()
                                         ↓
                                         robotRunner.run()
                                         ↓
               LowCmd ←──DDS──           publish LowCmd
  ↓
  ctrl[:] = LowCmd.motor_cmd[i].tau
  mj_step()  (next cycle)
```

The problem is that DDS is not shared memory. Each message must be:
1. Serialized (converted to bytes) by the sender.
2. Transmitted over a network socket (even on localhost).
3. Received and deserialized by the receiver.

This takes real time — measured in the log as `lowcmd_age_ms` (for LowCmd)
and `lowstate_age_ms` (for LowState). In a typical run, the round-trip is
**2–10 ms**. During locomotion the measured maximum was ~10 ms.

#### What "One Control Cycle of Latency" Means

The MuJoCo simulator and the MPC controller each run at a 5 ms period.
Because of DDS, the MPC never sees the *current* state — it always sees the
state from at least one 5 ms step ago.

```
Real time →  0ms    5ms    10ms   15ms   20ms   25ms
             │      │      │      │      │      │
Simulator:  [S0]───[S1]───[S2]───[S3]───[S4]───[S5]
              ↑      ↑      ↑      ↑      ↑      ↑
              publish LowState (current sensor readings)

DDS delay:        ←──2-10ms──→

MPC receives:          [S0]   [S1]   [S2]   [S3]   [S4]
                         ↑      ↑      ↑      ↑      ↑
                    MPC computes torques from 1-step-old state

MPC publishes LowCmd:  [T0]   [T1]   [T2]   [T3]   [T4]

DDS delay again:             ←──2-10ms──→

Simulator applies:             [T0]   [T1]   [T2]   [T3]
                                ↑      ↑      ↑      ↑
                           Torque T0 was computed from state S0,
                           but applies at time S2 (10ms later).
                           S2 is a completely different robot state.
```

The torque that arrives is 2 control cycles old. For a robot moving at speed,
2 cycles of 5 ms = 10 ms of position/velocity change that the torque did not
account for.

#### Why Stale Torques Are Especially Dangerous During Contact Transitions

The trot gait alternates between two diagonal pairs:

```
Phase A (0–100ms):   FL + RR in stance → FR + RL in swing
Phase B (100–200ms): FR + RL in stance → FL + RR in swing
```

During a contact transition (stance → swing), the MPC must:
- Stop pushing with the lifting foot immediately.
- Start the swing trajectory.
- Transfer all body support to the remaining stance legs.

This transition must happen within one or two control cycles (5–10 ms).
If the LowCmd delivering "stop pushing FL" arrives 10 ms late, FL continues
pushing for 2 extra cycles while FR+RL are already in the air. The body is
pushed sideways by an unsupported leg force.

```
IDEAL contact transition:

Time:         95ms     100ms    105ms
              │        │        │
FL contact:  [STANCE]─[LIFT]──[SWING]
FR contact:  [SWING] ─[LAND]──[STANCE]

ACTUAL with 10ms DDS latency:

Time:         95ms     100ms    105ms    110ms
              │        │        │        │
FL contact:  [STANCE]─[STANCE]─[LIFT]──[SWING]  ← 10ms too late
FR contact:  [SWING] ─[LAND] ─[STANCE]─[STANCE] ← already in stance
Body:        [OK]    ─[pushed sideways by FL] ← lateral fall begins
```

Even 1 cycle of delay (5 ms out of a 100 ms transition window) means 5%
of the gait cycle is using wrong support forces. For a 15 kg robot balanced on
two feet, this is enough to initiate a roll that the next MPC step cannot
fully correct.

#### Why the Direct MuJoCo Path Has Zero Latency

In `mujoco_mpc_locomotion.py`, the MPC controller runs inside the same process
as the simulator. There is no message passing:

```python
dof_states  = get_dof_state(data)    # reads data.sensordata directly
body_states = get_body_state(data)   # reads data.sensordata directly
legTorques  = robotRunner.run(...)   # computes immediately
data.ctrl[:] = legTorques[...]       # writes data.ctrl directly
mujoco.mj_step(model, data)         # steps simulator
```

The state read and the torque write happen in the same Python frame, with zero
DDS overhead. The MPC always works with the *current* sensor reading.

#### Fix B: Add Local kd Damping to the Motor Commands

Pure torque commands (`kp=0, kd=0, tau=X`) are open-loop: if the joint
overshoots because of a stale command, there is no corrective force.

Adding a local kd term (`kd=1.5`) makes each motor apply a damping force
proportional to its velocity:

```
τ_actual = τ_MPC - kd × dq
```

If a bad stale torque causes a joint to move too fast (`dq` large), the kd
term automatically counteracts it. This does not fix the latency, but it makes
the robot more forgiving of the 1–2 cycle lag. Think of it as adding shock
absorbers: the structure still transmits the impulse, but the oscillation is
damped before it compounds into a fall.

---

### Summary Table

| | Problem 1 (reset) | Problem 2 (DDS latency) |
|---|---|---|
| **Root cause** | StateEstimator memory is wiped before first MPC step | Torque commands arrive 5–10 ms after the state that generated them |
| **Failure mode** | MPC receives wrong contact phase → wrong support forces → immediate fall | Stale torques during gait transitions → lateral body push → roll |
| **When it hurts** | Only at the stand→move transition | Every gait cycle, worst at contact transitions |
| **Fix** | Remove `robotRunner.reset()` | Add kd damping; minimize DDS processing overhead |
| **Why fix works** | Estimator retains warm history from stand phase | kd damps joint velocity oscillations from stale commands |
| **Direct MuJoCo path** | Not affected (reset never called) | Not affected (zero-latency in-process reads) |
