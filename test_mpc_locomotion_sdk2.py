"""
test_mpc_locomotion_sdk2.py
===========================
Parameterised harness version of mpc_locomotion_sdk2.py for LATENCY / SPIKE testing.

It reproduces the 7 manual test configurations via a single --preset flag, so each
config can be run reproducibly N times WITHOUT editing source code. It also supports
--auto-move (warm up standing, then enter LOCOMOTION automatically, no keypress) and
--duration (run for a fixed time then exit cleanly), so an external script can launch
it 10x per config unattended.

Presets (match logs/1..6 and logs/normal):
  preset   mode        DDS Write   gc         phase
  1/normal 2-proc      ON          none       stand -> move
  2        2-proc      OFF         none       stand -> move
  3        2-proc      ON          none       stand only
  4        standalone  OFF         none       (auto) stand -> move
  5        standalone  ON          disable    (auto) stand -> move
  6        standalone  ON          none       (auto) stand -> move

2-proc presets (1,2,3,normal) need mujoco_sim_sdk2.py running separately.
standalone presets (4,5,6) need nothing else (dummy state is seeded internally).

Example:
  python test_mpc_locomotion_sdk2.py --preset 6 --duration 120 \
         --debug-log --debug-log-dir logs/suite/6/run_0
"""
import sys
import time
import os
import gc
import numpy as np
from argparse import ArgumentParser

from MPC_Controller.Parameters import Parameters
from MPC_Controller.robot_runner.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.utils import DTYPE
from mujoco_sim.pygame_gamepad import PyGamepad
from mujoco_sim.mujoco_sim_utils import *
from mujoco_sim.sdk2_debug_logger import CsvLogger, add_vec, vec_fields, wall_time_ns

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_ as HighState_default
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
from unitree_sdk2py.utils.crc import CRC
from mujoco_sim import config_sdk2 as config

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
PRESETS = {
    "1":      dict(standalone=False, no_write=False, gc_mode="none",    stand_only=False, auto_move=True),
    "normal": dict(standalone=False, no_write=False, gc_mode="none",    stand_only=False, auto_move=True),
    "2":      dict(standalone=False, no_write=True,  gc_mode="none",    stand_only=False, auto_move=True),
    "3":      dict(standalone=False, no_write=False, gc_mode="none",    stand_only=True,  auto_move=False),
    "4":      dict(standalone=True,  no_write=True,  gc_mode="none",    stand_only=False, auto_move=True),
    "5":      dict(standalone=True,  no_write=False, gc_mode="disable", stand_only=False, auto_move=True),
    "6":      dict(standalone=True,  no_write=False, gc_mode="none",    stand_only=False, auto_move=True),
}

parser = ArgumentParser(prog="test_mpc_locomotion_sdk2")
parser.add_argument("--preset", choices=list(PRESETS.keys()), default=None,
                    help="select a named test configuration (1,2,3,normal,4,5,6)")
# individual overrides (applied on top of preset)
parser.add_argument("--standalone", dest="standalone", action="store_true", default=None,
                    help="run with dummy state, no bridge/robot")
parser.add_argument("--no-write", dest="no_write", action="store_true", default=None,
                    help="skip the actual DDS publish (still computes crc + logs)")
parser.add_argument("--gc-mode", choices=["none", "disable", "freeze"], default=None,
                    help="garbage-collector handling around the loop")
parser.add_argument("--stand-only", dest="stand_only", action="store_true", default=None,
                    help="never enter locomotion (stand the whole time)")
parser.add_argument("--auto-move", dest="auto_move", action="store_true", default=None,
                    help="after --warmup seconds of standing, enter LOCOMOTION automatically")
parser.add_argument("--warmup", type=float, default=5.0,
                    help="seconds to stand before auto-move (default 5)")
parser.add_argument("--duration", type=float, default=120.0,
                    help="run this many seconds then exit cleanly (0 = forever)")
parser.add_argument("--debug-log", action="store_true", help="write debug CSV logs")
parser.add_argument("--debug-log-dir", default="logs/suite/run", help="directory for debug CSV logs")
args = parser.parse_args()

# resolve config: start from preset (if any), apply explicit overrides
cfg = dict(standalone=False, no_write=False, gc_mode="none", stand_only=False, auto_move=True)
if args.preset is not None:
    cfg.update(PRESETS[args.preset])
for k in ("standalone", "no_write", "gc_mode", "stand_only", "auto_move"):
    v = getattr(args, k)
    if v is not None:
        cfg[k] = v

dt = Parameters.controller_dt
gamepad = PyGamepad()

print(f"[TEST] preset={args.preset} cfg={cfg} warmup={args.warmup}s duration={args.duration}s")


def _get_mpc_snapshot(cMPC, se):
    if cMPC.firstRun:
        return {}
    mpc_x = np.concatenate([
        se.rpyBody.flatten(), se.position.flatten(),
        se.omegaBody.flatten(), se.vBody.flatten(),
    ])
    mpc_x_des = np.array([
        0, 0, 0,
        0, 0, float(cMPC._body_height),
        0, 0, float(cMPC._yaw_turn_rate),
        float(cMPC._x_vel_des), float(cMPC._y_vel_des), 0,
    ])
    return {
        "mpc_x": mpc_x, "mpc_x_des": mpc_x_des,
        "mpc_u_grf": cMPC.f_ff.reshape(12),
        "foot_p":     np.array([cMPC.pFoot[i].flatten() for i in range(4)]).flatten(),
        "foot_p_des": np.array([cMPC.footSwingTrajectories[i].getPosition().flatten() for i in range(4)]).flatten(),
        "foot_v_des": np.array([cMPC.footSwingTrajectories[i].getVelocity().flatten() for i in range(4)]).flatten(),
    }


class TestMPCLocomotion:
    def __init__(self):
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None
        self.high_state = None
        self.debug_logger = None
        self.pub_logger = None
        self.lowcmd_pub_seq = 0
        self.crc = CRC()

    def Init(self):
        self.InitLowCmd()
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)
        self.highstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.highstate_subscriber.Init(self.HighStateMessageHandler, 10)

    def InitLowCmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01
            self.low_cmd.motor_cmd[i].q = 2.146e9
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = 16000
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    # CORRECT handlers: store the real incoming message
    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

    def HighStateMessageHandler(self, msg: SportModeState_):
        self.high_state = msg

    def seed_dummy_state(self):
        """Standalone: valid standing pose so the loop runs with no bridge/robot."""
        self.low_state = LowState_default()
        self.high_state = HighState_default()
        self.low_state.imu_state.quaternion[0] = 1.0  # identity (w,x,y,z) -> no NaN
        self.low_state.imu_state.quaternion[1] = 0.0
        self.low_state.imu_state.quaternion[2] = 0.0
        self.low_state.imu_state.quaternion[3] = 0.0
        for i in range(12):
            self.low_state.motor_state[i].q = float(STAND_TARGET[i])
            self.low_state.motor_state[i].dq = 0.0
        self.high_state.position[2] = 0.30

    def LowCmdWrite(self):
        now_ns = wall_time_ns()
        self.lowcmd_pub_seq += 1
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        if not cfg["no_write"]:
            self.lowcmd_publisher.Write(self.low_cmd)
        if self.pub_logger is not None:
            row = {"wall_time_ns": now_ns, "pub_seq": self.lowcmd_pub_seq, "crc": self.low_cmd.crc}
            add_vec(row, "tau_cmd_FR", [self.low_cmd.motor_cmd[i].tau for i in range(3)], 3)
            add_vec(row, "tau_cmd_RL", [self.low_cmd.motor_cmd[i].tau for i in range(9, 12)], 3)
            add_vec(row, "q_cmd_FR",   [self.low_cmd.motor_cmd[i].q for i in range(3)], 3)
            add_vec(row, "q_cmd_RL",   [self.low_cmd.motor_cmd[i].q for i in range(9, 12)], 3)
            self.pub_logger.write(row)

    def MPC_RUN(self):
        robotRunner = RobotRunnerFSM()
        robotRunner.init(RobotType.GO2)

        running_time = 0.0
        legTorques = np.zeros(12, dtype=DTYPE)
        commands = np.zeros(3, dtype=DTYPE)
        waiting_for_states = True

        if args.debug_log:
            self._init_debug_loggers(args.debug_log_dir)

        # standalone: seed state so we never block waiting for a bridge
        if cfg["standalone"]:
            self.seed_dummy_state()

        # start standing
        gamepad._enter_stand()

        # gc policy
        if cfg["gc_mode"] in ("disable", "freeze"):
            gc.collect()
            gc.disable() if cfg["gc_mode"] == "disable" else gc.freeze()

        t_start = time.time()
        moved = False
        try:
            while True:
                step_start = time.time()
                elapsed = step_start - t_start

                # exit after duration
                if args.duration > 0 and elapsed > args.duration:
                    break

                # auto-move after warmup
                if cfg["auto_move"] and not cfg["stand_only"] and not moved and elapsed > args.warmup:
                    gamepad._enter_move()
                    moved = True
                    print(f"[TEST] auto-move at t={elapsed:.1f}s")

                commands[:] = 0.0
                running_time += dt

                # 2-proc: wait for first real state
                if self.low_state is None or self.high_state is None:
                    if waiting_for_states:
                        print("Waiting for rt/lowstate and rt/sportmodestate...")
                        waiting_for_states = False
                    time.sleep(dt)
                    continue

                dof_states = get_dof_state_sdk2(self.low_state)
                body_states = get_body_state_sdk2(self.low_state, self.high_state)

                if gamepad.is_standing:
                    self.low_cmd = pd_stand_sdk2(self.low_state, self.low_cmd, running_time)
                    robotRunner._legController.updateData(dof_states)
                    robotRunner._stateEstimator.update(body_states)

                if gamepad.is_moving:
                    lin_speed, ang_speed, e_stop = gamepad.get_command()
                    Parameters.cmpc_gait = gamepad.get_gait()
                    Parameters.control_mode = gamepad.get_mode()
                    if not e_stop:
                        commands[0] = lin_speed[0]
                        commands[1] = lin_speed[1]
                        commands[2] = ang_speed
                    legTorques = robotRunner.run(dof_states, body_states, commands).astype(np.float32)
                    for i in range(12):
                        j = LEG_MJC_TO_MPC[i]
                        self.low_cmd.motor_cmd[i].q = 2.146e9
                        self.low_cmd.motor_cmd[i].kp = 0.0
                        self.low_cmd.motor_cmd[i].dq = 0.0
                        self.low_cmd.motor_cmd[i].kd = 2.0
                        self.low_cmd.motor_cmd[i].tau = legTorques[j]

                if self.debug_logger is not None:
                    self._log_mpc_debug(robotRunner, running_time, commands, legTorques)

                if Parameters.locomotionUnsafe:
                    gamepad.fake_event(ev_type='Key', code='BTN_TR', value=0)
                    Parameters.locomotionUnsafe = False

                self.LowCmdWrite()

                time_until_next_step = dt - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        finally:
            if cfg["gc_mode"] == "disable":
                gc.enable()
            if self.debug_logger is not None:
                self.debug_logger.close()
            if self.pub_logger is not None:
                self.pub_logger.close()
            print(f"[TEST] done, ran {time.time()-t_start:.1f}s")

    def _init_debug_loggers(self, log_dir):
        os.makedirs(log_dir, exist_ok=True)
        mpc_fields = (
            ["wall_time_ns", "running_time", "cmd_vx", "cmd_vy", "cmd_wz"]
            + vec_fields("low_q_FR", 3) + vec_fields("low_q_RL", 3)
            + vec_fields("low_dq_FR", 3) + vec_fields("low_dq_RL", 3)
            + vec_fields("tau_FR", 3) + vec_fields("tau_RL", 3)
            + vec_fields("imu_quat", 4) + vec_fields("imu_gyro", 3)
            + vec_fields("high_pos", 3) + vec_fields("high_vel", 3)
            + vec_fields("se_rpy", 3) + vec_fields("se_pos", 3)
            + vec_fields("se_vbody", 3) + vec_fields("se_omega_body", 3)
            + vec_fields("mpc_x", 12) + vec_fields("mpc_x_des", 12) + vec_fields("mpc_u_grf", 12)
            + vec_fields("foot_p_FR", 3) + vec_fields("foot_p_des_FR", 3) + vec_fields("foot_v_des_FR", 3)
            + vec_fields("foot_p_RL", 3) + vec_fields("foot_p_des_RL", 3) + vec_fields("foot_v_des_RL", 3)
        )
        pub_fields = (
            ["wall_time_ns", "pub_seq", "crc"]
            + vec_fields("tau_cmd_FR", 3) + vec_fields("tau_cmd_RL", 3)
            + vec_fields("q_cmd_FR", 3) + vec_fields("q_cmd_RL", 3)
        )
        self.debug_logger = CsvLogger(os.path.join(log_dir, "controller_mpc.csv"), mpc_fields)
        self.pub_logger = CsvLogger(os.path.join(log_dir, "controller_lowcmd_pub.csv"), pub_fields)
        print(f"[TEST] writing logs to {log_dir}")

    def _log_mpc_debug(self, robotRunner, running_time, commands, legTorques):
        se = robotRunner._stateEstimator.getResult()
        cMPC = robotRunner._controlFSM.statesList.locomotion.cMPC
        snap = _get_mpc_snapshot(cMPC, se)
        row = {
            "wall_time_ns": wall_time_ns(), "running_time": running_time,
            "cmd_vx": float(commands[0]), "cmd_vy": float(commands[1]), "cmd_wz": float(commands[2]),
        }
        add_vec(row, "low_q_FR",  [self.low_state.motor_state[i].q for i in range(3)], 3)
        add_vec(row, "low_q_RL",  [self.low_state.motor_state[i].q for i in range(9, 12)], 3)
        add_vec(row, "low_dq_FR", [self.low_state.motor_state[i].dq for i in range(3)], 3)
        add_vec(row, "low_dq_RL", [self.low_state.motor_state[i].dq for i in range(9, 12)], 3)
        add_vec(row, "tau_FR", legTorques[3:6], 3)
        add_vec(row, "tau_RL", legTorques[6:9], 3)
        add_vec(row, "imu_quat", self.low_state.imu_state.quaternion, 4)
        add_vec(row, "imu_gyro", self.low_state.imu_state.gyroscope, 3)
        add_vec(row, "high_pos", self.high_state.position, 3)
        add_vec(row, "high_vel", self.high_state.velocity, 3)
        add_vec(row, "se_rpy", se.rpy.flatten(), 3)
        add_vec(row, "se_pos", se.position.flatten(), 3)
        add_vec(row, "se_vbody", se.vBody.flatten(), 3)
        add_vec(row, "se_omega_body", se.omegaBody.flatten(), 3)
        add_vec(row, "mpc_x",     snap.get("mpc_x", []), 12)
        add_vec(row, "mpc_x_des", snap.get("mpc_x_des", []), 12)
        add_vec(row, "mpc_u_grf", snap.get("mpc_u_grf", []), 12)
        fp = snap.get("foot_p", []); fpdes = snap.get("foot_p_des", []); fvdes = snap.get("foot_v_des", [])
        add_vec(row, "foot_p_FR",     fp[3:6]    if len(fp)    >= 6 else [], 3)
        add_vec(row, "foot_p_RL",     fp[6:9]    if len(fp)    >= 9 else [], 3)
        add_vec(row, "foot_p_des_FR", fpdes[3:6] if len(fpdes) >= 6 else [], 3)
        add_vec(row, "foot_p_des_RL", fpdes[6:9] if len(fpdes) >= 9 else [], 3)
        add_vec(row, "foot_v_des_FR", fvdes[3:6] if len(fvdes) >= 6 else [], 3)
        add_vec(row, "foot_v_des_RL", fvdes[6:9] if len(fvdes) >= 9 else [], 3)
        self.debug_logger.write(row)


if __name__ == "__main__":
    try:
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        node = TestMPCLocomotion()
        node.Init()
        node.MPC_RUN()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        gamepad.stop()
        print("[TEST] exit")
