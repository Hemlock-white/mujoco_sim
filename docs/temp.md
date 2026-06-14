# Technical Notes — MPC Locomotion

---

## 1. `gym.get_actor_dof_state` / `get_actor_rigid_body_state` vs `get_dof_state` / `get_body_state`

### Isaac Gym API

**`gym.get_actor_dof_state(env_handle, actor_handle, gymapi.STATE_ALL)`**

Returns a numpy array of shape `(num_dof,)` where each element is a struct `{pos: float32, vel: float32}`. The order of joints follows the **URDF/asset DOF order** — for Go2 this is typically:

```
index  joint
0-2    FL hip, thigh, calf
3-5    FR hip, thigh, calf
6-8    RL hip, thigh, calf
9-11   RR hip, thigh, calf
```

This is FL-first, which happens to match MPC internal order. The API internally does a GPU→CPU tensor copy, so it is relatively expensive and should not be called every substep in a vectorised environment.

**`gym.get_actor_rigid_body_state(env_handle, actor_handle, gymapi.STATE_ALL)`**

Returns a numpy array of shape `(num_bodies,)` where each element is a struct `{pose: {p: vec3, r: quat}, vel: {linear: vec3, angular: vec3}}`. This includes **every rigid link** — the floating base body plus all 12 limb links (hip, thigh, calf × 4) = 13 bodies. Order follows the URDF link order. You must look up the body index with `gym.find_actor_rigid_body_handle(env, actor, "body_name")` to get the base body specifically.

Key difference from DOF state: it gives you the full spatial pose and velocity of every link, not just joint angles. This is what you use to get the base body's world-frame position and orientation — something DOF state alone cannot give you.

---

### This Project's Functions (MuJoCo)

**`get_dof_state(data)` — `mujoco_sim_utils.py:39`**

Reads from `data.sensordata[0:12]` (joint positions) and `data.sensordata[12:24]` (joint velocities). These are populated by MuJoCo's joint-position and joint-velocity sensors in the order they appear in the XML scene file, which for this Go2 model is **FR-first**:

```
sensordata index   joint
0-2                FR hip, thigh, calf
3-5                FL hip, thigh, calf
6-8                RR hip, thigh, calf
9-11               RL hip, thigh, calf
```

The permutation array `LEG_MJC_TO_MPC = [3,4,5, 0,1,2, 9,10,11, 6,7,8]` reorders to MPC FL-first order during the read loop. The output is the same `{pos, vel}` struct format that Isaac Gym returns, just derived differently.

**`get_body_state(data)` — `mujoco_sim_utils.py:53`**

Reads the floating base from fixed sensor offsets hardcoded in `data.sensordata`:

```
sensordata[36:40]  imu_quat  (w, x, y, z)
sensordata[40:43]  imu_gyro  (body-frame angular velocity)
sensordata[46:49]  frame_pos (world-frame body position)
sensordata[49:52]  frame_vel (world-frame linear velocity)
```

There is no body-index lookup — the addresses are fixed by the sensor order defined in `scene.xml`. The gyroscope reads body-frame ω, which is then rotated to world frame: `ω_world = R_body_to_world @ ω_body`. Isaac Gym's `get_actor_rigid_body_state` returns world-frame angular velocity directly, so the rotation is done there by the simulator itself.

---

### Summary Table

| | Isaac Gym `get_actor_dof_state` | Isaac Gym `get_actor_rigid_body_state` | `get_dof_state` (MuJoCo) | `get_body_state` (MuJoCo) |
|---|---|---|---|---|
| What it returns | 12 joint {pos, vel} | All rigid bodies {pose, vel} | 12 joint {pos, vel} | Base {pose, vel} |
| Includes base body? | No | Yes (index 0) | No | Yes (only base) |
| Native order | URDF/asset order (FL-first) | URDF link order | MuJoCo sensor order (FR-first), permuted to FL-first | Fixed sensordata offsets |
| Velocity frame | N/A (angles) | World frame | N/A (angles) | Angular: world frame (after rotation) |
| Cost | GPU→CPU copy | GPU→CPU copy | Cheap (sensor array read) | Cheap (sensor array read) |

---

## 2. Why Waiting Longer After Stand Reduces the Dip/Wobble on Mode Switch

### Mechanical Settling

When `pd_stand` finishes bringing the robot up, the legs have spring-like compliance. Even after the target is reached, the body oscillates at small amplitude around the equilibrium — like a mass-spring system that hasn't fully damped. If MPC takes over during this oscillation, it observes a non-zero `vBody` from the SE, treats it as a real velocity command error, and produces corrective torques that can amplify the oscillation rather than damp it.

After a few more seconds, the PD controller has fully damped these oscillations. The robot is in a mechanically stable, near-zero-velocity state. MPC's first computation starts from a clean initial condition.

### State Estimator Warm-Up

During `is_standing` the code calls:

```python
robotRunner._legController.updateData(dof_states)
robotRunner._stateEstimator.update(body_states)
```

The state estimator (complementary filter / Kalman filter) is recursive — its estimate quality depends on how many observations it has accumulated. On the first few calls the velocity estimate has high variance and can read non-zero even on a stationary robot due to gyroscope bias and numerical integration errors. After ~0.5–2 seconds of updates, the filter converges and gives a stable zero-velocity estimate.

When the FSM transitions to LOCOMOTION it calls `stateEstimator.reset()` (inside `FSM_State_Locomotion.onEnter()`), discarding the warm history. However, the **leg controller data is not reset** — it still holds the correct standing joint states from the warm-up period. This means the MPC immediately has accurate foot position estimates via forward kinematics, even though the velocity estimate starts fresh.

If you skip the warm-up, `reset()` starts the SE from a cold state AND the leg controller has no history — both inputs to the first MPC solve are inaccurate simultaneously.

### `locomotionSafe()` Check

The FSM will not stay in LOCOMOTION if `locomotionSafe()` fails. The check includes:

```
|rpy[0]| < 40°     (roll)
|rpy[1]| < 40°     (pitch)
p_leg[2] <= 0      (foot at or below hip)
|p_leg[1]| <= 0.18 (foot not too far laterally)
```

During mechanical settling, brief roll/pitch oscillations can momentarily exceed the threshold, causing the FSM to bounce back to RECOVERY_STAND. Each bounce calls `onEnter()` again, resetting the SE, and `run()` is never called during the TRANSITIONING mode, so torques are zero. This is the 18.7-second zero-torque window observed in the SDK2 logs. Waiting for full mechanical settling eliminates these false-positive safety failures.

### In Short

| Wait too short | Wait long enough |
|---|---|
| Robot still oscillating | Body at rest |
| SE has unconverged velocity | SE has stable (near-zero) velocity |
| First MPC solve sees bad state | First MPC solve sees clean state |
| `locomotionSafe()` may fail → FSM bounces → τ=0 | `locomotionSafe()` passes immediately |
| Dip, sudden jerk, possible collapse | Smooth acceleration |

---

## 3. How Debug Logging and Latency Plots Work

### Three Log Files

All logs live in `logs/sdk2_debug/` (default) or the path passed to `--debug-log-dir`.

**`controller_mpc.csv`** — one row per MPC main loop iteration (nominal 5 ms / 200 Hz)

Written by `_log_mpc_debug()` called at the end of each `MPC_RUN` while loop. Key columns:

| Column | Meaning |
|---|---|
| `wall_time_ns` | Absolute system wall clock in nanoseconds when the row was written (`time.time_ns()`) |
| `running_time` | Logical elapsed time (incremented by `dt` each loop, not measured) |
| `cmd_vx/vy/wz` | Gamepad velocity commands that cycle |
| `low_q_FR_0/1/2` | Raw motor joint angles from `low_state.motor_state[0/1/2].q` (SDK2 FR = motors 0-2) |
| `low_q_RL_0/1/2` | Motors 9-11 |
| `tau_FR_0/1/2` | MPC output torques in MPC order (MPC FR = indices 3-5 of `legTorques`) |
| `imu_quat_*` | Raw quaternion from IMU |
| `imu_gyro_*` | Raw gyroscope (rad/s, body frame) |
| `high_pos/vel_*` | SportModeState position and velocity (from Go2's onboard estimator) |
| `se_rpy_*` | Roll/pitch/yaw from the MPC state estimator (rad) |
| `se_vbody_*` | Body-frame velocity from SE (m/s) |
| `mpc_x_0..11` | Full MPC state vector: `[rpy(3), pos(3), omega(3), vbody(3)]` |
| `mpc_x_des_0..11` | Desired state: `[0,0,0, 0,0,h, 0,0,wz_des, vx_des,vy_des,0]` |
| `mpc_u_grf_0..11` | Ground reaction force outputs in MPC leg order (FL, FR, RL, RR × xyz) |
| `foot_p_FR/RL_*` | Actual foot world-frame positions (from `cMPC.pFoot`) |
| `foot_p_des_FR/RL_*` | Desired swing trajectory positions |

**`controller_lowcmd_pub.csv`** — one row per `LowCmdWrite` thread tick (nominal 2 ms / 500 Hz)

Written at the moment the command is sent over DDS. Contains `wall_time_ns`, sequence number, CRC, and commanded `tau`/`q` for FR and RL motors.

**`bridge_sim.csv`** — from the MuJoCo bridge (only when using `mujoco_sim_sdk2.py`)

Contains `lowcmd_age_ms`: the time between when the controller published the LowCmd and when the bridge received and applied it. This measures DDS round-trip latency.

---

### Latency Plots (`latency_timing.png`)

Three panels, computed in `plot_latency()`:

**Panel 1 — `bridge lowcmd age ms`**

`bridge_sim.csv["lowcmd_age_ms"]` plotted directly. This is the one-way delay from controller to bridge. In simulation on one machine this should be <1 ms. On real hardware over a physical network it can be 2–10 ms depending on DDS configuration and network load.

**Panel 2 — `controller lowcmd publish dt ms`**

```python
dt_ms = np.diff(wall_time_ns_pub) / 1e6
```

Consecutive differences between publish timestamps. Should be a flat line at 2 ms. What to look for:
- **Steady value above 2 ms**: the publish thread is not being scheduled at 500 Hz — OS is deprioritising it.
- **Occasional large spikes**: the thread was preempted, likely by the MPC thread.
- **Values below 2 ms**: can't happen with `RecurrentThread` unless the interval was changed.

**Panel 3 — `controller MPC loop dt ms`**

```python
dt_ms = np.diff(wall_time_ns_ctrl) / 1e6
```

Consecutive differences between MPC loop rows. Should be a flat line at 5 ms. What to look for:
- **Consistently near 5 ms**: healthy — computation fits the budget.
- **Spikes to 10–20 ms**: one iteration overran; `time.sleep(dt)` was effectively 0 because computation already consumed >5 ms. The SE integrated for ~10 ms instead of 5 ms.
- **Values of 0–1 ms intermixed with 10 ms values**: the sleep is being bypassed; loop is spinning.

---

### How to Read the Data Together

1. Open `latency_timing.png` first. If MPC dt is consistently above 5 ms → the Orin CPU is too slow for this rate. If it's fine on PC but bad on Orin → you have a deployment compute budget problem (see Section 4).

2. Open `imu_state_estimator.png`. Compare `se_rpy` (state estimator output) with `imu_quat` (raw sensor). If they diverge, the SE is running at the wrong rate. Look at the velocity subplot (`se_vbody`): a non-zero velocity on a stationary robot indicates SE has not converged.

3. Open `mpc_state_input.png`. Compare `mpc_x_des` (dashed) with `mpc_x` (solid) for velocity components (indices 9-11). A persistent large gap means the MPC desired state is being set correctly but the robot is not tracking — either torque is too small or the SE is giving wrong feedback.

4. Open `foot_actual_desired.png`. The z-axis plot is most diagnostic: if `foot_p_des_z` swings between 0 and -0.1 m (swing arc) but `foot_p_z` stays flat, the foot is not actually leaving the ground — likely a torque magnitude issue or hardware limit.

---

## 4. Lecture: Latency, Real-Time Control, and Deployment on the Orin Nano

### 4.1 The Control Loop as a Time Contract

A robot controller is a **time-critical feedback loop**. At each timestep $k$, it must:

1. Read sensor state $x_k$
2. Compute control output $u_k$
3. Actuate $u_k$ before step $k+1$ begins

If any step is late, the actuator receives a command computed from stale state. This is equivalent to adding an unknown delay to the plant — it shifts phase margin, destabilises the loop, and in the worst case causes the robot to receive torques computed for a configuration it is no longer in.

This MPC controller targets $dt = 5$ ms (200 Hz). The real hardware runs its low-level position/torque loop at 500 Hz (2 ms). So the MPC must fit **all of Python overhead + OSQP QP solve + DDS publish within 5 ms** to maintain the contract.

### 4.2 Why It Works on Your PC but Collapses on the Orin

**CPU speed.** The Orin Nano has a 6-core ARM Cortex-A78AE running at ~1.5 GHz. Your development PC likely has a 6+ core x86 at 3–5 GHz with much higher single-thread IPC. Python + OSQP single-thread performance scales roughly with clock speed and IPC. A solve that takes 1 ms on the PC can take 4–6 ms on the Orin.

**Python GIL and thread model.** The controller runs three concurrent threads:
- MPC main loop (200 Hz)
- `LowCmdWrite` publish thread (500 Hz)
- State subscriber callbacks (DDS reader threads)

All Python threads share the GIL. When the MPC loop is computing, subscriber callbacks are blocked. If a LowState arrives during an OSQP solve, it is queued but not processed. The next MPC iteration then reads a LowState that is 1–2 loop periods old.

**`time.sleep(dt)` is not a real-time primitive.** On Linux, `time.sleep(0.005)` asks the OS scheduler "wake me after 5 ms at the earliest." The actual wake-up can be 1–10 ms later depending on scheduler load, timer resolution, and process priority. On a loaded Orin (running DDS, Python, potentially display), this jitter can be significant.

**The collapse cascade:**

```
MPC solve overruns 5 ms
→ time.sleep(0) (sleep of negative slack → immediate)
→ next iteration starts immediately, no gap
→ running_time accumulates correctly but wall clock is ahead
→ OR: sleep overshoots → loop period becomes 8-10 ms
→ SE integrates at wrong rate → velocity estimate drifts
→ gait phase timer runs at wrong speed → foot swing timing wrong
→ stance/swing assignment mismatched with actual foot contact
→ MPC applies swing-phase GRF to a foot that is still on ground
→ torques are wrong → robot stumbles → falls
```

### 4.3 How to Diagnose Which Part Is Slow

Add timing around each section of `MPC_RUN`:

```python
t0 = time.perf_counter()
dof_states   = get_dof_state_sdk2(self.low_state)
body_states  = get_body_state_sdk2(self.low_state, self.high_state)
t1 = time.perf_counter()

legTorques   = robotRunner.run(dof_states, body_states, commands)
t2 = time.perf_counter()

# (LowCmd is sent by LowCmdWrite thread separately)

print(f"read={1000*(t1-t0):.2f}ms  mpc={1000*(t2-t1):.2f}ms")
```

The existing `Controller run time` print already measures `robotRunner.run()`. Look at that number on the Orin. If it is:

- **<3 ms**: MPC is fine; the problem is DDS overhead, GIL contention, or sleep jitter. Try reducing DDS history depth or pinning threads to cores.
- **3–5 ms**: marginal; use real-time scheduling (`chrt -f 50 python3 ...`) to reduce sleep jitter and you may be fine.
- **>5 ms**: the MPC compute itself is too slow. Options: increase `dt` to 10 ms (100 Hz), reduce MPC horizon `N`, or compile OSQP with NEON optimisation.

### 4.4 Fixes in Priority Order

**Fix 1 — Deadline-based sleep instead of fixed sleep.**

Replace:
```python
time.sleep(dt)
```
With:
```python
loop_end = loop_start + dt
gap = loop_end - time.perf_counter()
if gap > 0:
    time.sleep(gap)
loop_start = time.perf_counter()
```

This keeps the period correct even when computation varies. A slow iteration steals time from the sleep, not from the next iteration's sensor reading window.

**Fix 2 — Real-time scheduling priority.**

Run with FIFO real-time priority on the Orin:
```bash
sudo chrt -f 50 python3 mpc_locomotion_sdk2.py
```

Or set the process nice value:
```bash
sudo nice -n -20 python3 mpc_locomotion_sdk2.py
```

This prevents the OS from preempting the MPC loop to service other processes mid-solve.

**Fix 3 — Reduce the loop rate if compute budget is exceeded.**

Change `Parameters.controller_dt` to `0.01` (100 Hz). The MPC was originally designed for 500 Hz hardware but the Python wrapper already runs at 200 Hz and works well. 100 Hz gives the OSQP solver twice the budget. The gait timing, SE, and swing trajectories all use `dt` internally so changing it propagates correctly.

**Fix 4 — Avoid redundant DDS traffic.**

The bridge publishes `LowState` and `HighState` at the MuJoCo simulation step rate. On the real hardware both come from the robot. Ensure the DDS history depth and reliability QoS are set to `KEEP_LAST 1` / `BEST_EFFORT`. Buffering more than 1 sample wastes memory and adds processing time in the subscriber callback.

**Fix 5 — Profile and eliminate Python overhead hot spots.**

On the Orin, use `cProfile`:
```bash
python3 -m cProfile -s cumtime mpc_locomotion_sdk2.py 2>&1 | head -40
```

Common hot spots in this codebase: numpy array allocation inside the loop (use pre-allocated buffers), repeated attribute lookups (`self.low_state.motor_state[i].q` inside a Python for-loop), and the `add_vec` debug logging (disable with `--debug-log` off during timing tests).

### 4.5 The Bigger Picture: Sim-to-Real Latency Gap

Even when the compute budget is met, deploying from PC-sim to Orin-hardware introduces a new latency source: **DDS network delay**. In simulation, controller and bridge run on the same machine and the LowCmd is applied in the same process step. On hardware, the LowCmd travels:

```
Orin (controller) → Ethernet → Go2 body computer → motor drive MCU
```

Each hop adds ~1–3 ms. The motor MCU applies the torque at its next 2 ms control tick. So the total actuator delay is **2–8 ms**, compared to near-zero in simulation.

This extra delay reduces phase margin. If the controller was tuned at sim latency of ~0 ms, the real robot may oscillate or diverge at hardware latency of 4–8 ms. The standard mitigation is to either:
- Add a simulated actuator delay to the sim (add `N_delay` steps of command buffering in the bridge), or
- Re-tune `kd` (damping) downward slightly to reduce sensitivity to velocity feedback that arrives late.

### 4.6 Summary Checklist for Orin Deployment

```
[ ] Measure robotRunner.run() wall time on Orin (target: < 3 ms)
[ ] Use deadline-based sleep (not fixed time.sleep)
[ ] Run with sudo chrt -f 50 or nice -n -20
[ ] Disable --debug-log during timing validation (CSV writes add latency)
[ ] If run time > 5 ms: increase dt to 0.01 or reduce MPC horizon
[ ] Verify DDS QoS: KEEP_LAST 1, BEST_EFFORT
[ ] After timing is stable: check latency plots for MPC dt consistency
[ ] Compare se_vbody on Orin vs PC logs to catch SE drift
[ ] Add sim actuator delay to bridge if real robot is more oscillatory than sim
```
