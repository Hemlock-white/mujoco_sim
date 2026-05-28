"""MPC Locomotion related headers and functions"""
import sys
import time
import os
import numpy as np
from MPC_Controller.Parameters import Parameters
from MPC_Controller.robot_runner.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.utils import DTYPE
from mujoco_sim.pygame_gamepad import PyGamepad
#from mujoco_sim import udp_reader
#import mujoco
from mujoco_sim.mujoco_sim_utils import *
from mujoco_sim.sdk2_debug_logger import CsvLogger, add_vec, vec_fields, wall_time_ns
from argparse import ArgumentParser

"""sdk related libraries"""
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from mujoco_sim import config_sdk2 as config

""""""
parser = ArgumentParser(prog="mpc_locomotion_sdk2")
parser.add_argument("--disable-gamepad", action="store_true")
parser.add_argument("--enable-motion-switcher", action="store_true")
parser.add_argument("--debug-log", action="store_true", help="write SDK2/MPC debug CSV logs")
parser.add_argument("--debug-log-dir", default="logs/sdk2_debug", help="directory for debug CSV logs")
# 移除了 num-envs，專注於模擬單一機器人
args = parser.parse_args()
use_gamepad = not args.disable_gamepad
if use_gamepad:
    gamepad = PyGamepad()
    #gamepad = udp_reader.UDPGamepad(port=9876)

dt = Parameters.controller_dt



def _get_mpc_snapshot(cMPC, se):
    if cMPC.firstRun:
        return {}
    gait_map = {
        0: cMPC.trotting, 1: cMPC.bounding, 2: cMPC.pronking,
        3: cMPC.pacing,   5: cMPC.galloping, 6: cMPC.walking, 7: cMPC.trotRunning,
    }
    gait = gait_map.get(cMPC.current_gait, cMPC.trotting)
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
        "mpc_x":         mpc_x,
        "mpc_x_des":     mpc_x_des,
        "mpc_u_grf":     cMPC.f_ff.reshape(12),
        "foot_p":        np.array([cMPC.pFoot[i].flatten() for i in range(4)]).flatten(),
        "foot_p_des":    np.array([cMPC.footSwingTrajectories[i].getPosition().flatten() for i in range(4)]).flatten(),
        "foot_v_des":    np.array([cMPC.footSwingTrajectories[i].getVelocity().flatten() for i in range(4)]).flatten(),
        "contact_state": gait.getContactState().flatten(),
        "swing_state":   gait.getSwingState().flatten(),
        "mpc_table":     list(gait.getMpcTable()),
    }

class MPCLocomotionSDK2:
    def __init__(self):
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None
        self.high_state = None
        self.debug_logger = None
        self.pub_logger = None
        self.lowcmd_pub_seq = 0
        # thread handling
        self.lowCmdWriteThreadPtr = None
        self.crc = CRC()

    def Init(self):
        self.InitLowCmd()

        # create publisher #
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_) #mujoco_sdk2.py subscribe to rt/lowcmd
        self.lowcmd_publisher.Init()

        # create subscriber #
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_) #mujoco_sdk2.py publish to rt/lowstate
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)
        self.highstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_) #mujoco_sdk2.py publish to rt/sportmodestate
        self.highstate_subscriber.Init(self.HighStateMessageHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=0.002, target=self.LowCmdWrite, name="writebasiccmd"
        )
        self.lowCmdWriteThreadPtr.Start()

    # Private methods
    def InitLowCmd(self):
        self.low_cmd.head[0]=0xFE
        self.low_cmd.head[1]=0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= 2.146e9
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = 16000
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

    def HighStateMessageHandler(self, msg: SportModeState_):
        self.high_state = msg

    def LowCmdWrite(self):
        now_ns = wall_time_ns()
        self.lowcmd_pub_seq += 1
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)
        if self.pub_logger is not None:
            row = {
                "wall_time_ns": now_ns,
                "pub_seq": self.lowcmd_pub_seq,
                "crc": self.low_cmd.crc,
            }
            add_vec(row, "tau_cmd_FR", [self.low_cmd.motor_cmd[i].tau for i in range(3)],     3)
            add_vec(row, "tau_cmd_RL", [self.low_cmd.motor_cmd[i].tau for i in range(9, 12)], 3)
            add_vec(row, "q_cmd_FR",   [self.low_cmd.motor_cmd[i].q   for i in range(3)],     3)
            add_vec(row, "q_cmd_RL",   [self.low_cmd.motor_cmd[i].q   for i in range(9, 12)], 3)
            self.pub_logger.write(row)
        """
        //FR_ 0->0, FR_ 1->1, FR_ 2->2 motor sequence, currently only 12 motors are used, later reserved.
        //FL_ 0->3, FL_ 1->4, FL_ 2->5
        //RR_ 0->6, RR_ 1->7, RR_ 2->8
        //RL_ 0->9, RL_ 1->10, RL_ 2->11
        """

    # Set up MPC controller
    def MPC_RUN(self):
        global use_gamepad, gamepad
        robotRunner = RobotRunnerFSM()
        robotRunner.init(RobotType.GO2)

        running_time = 0.0
        legTorques = np.zeros(12, dtype=DTYPE)
        waiting_for_states = True

        if args.debug_log:
            self._init_debug_loggers(args.debug_log_dir)

        while True:
            commands = np.zeros(3, dtype=DTYPE)
            running_time += dt

            if not use_gamepad:
                break

            if self.low_state is None or self.high_state is None:
                if waiting_for_states:
                    print("Waiting for rt/lowstate and rt/sportmodestate from mujoco_sim_sdk2.py...")
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
                    commands = np.array([lin_speed[0], lin_speed[1], ang_speed], dtype=DTYPE)

                # run controllers
                legTorques = robotRunner.run(dof_states, body_states, commands).astype(np.float32)

                for i in range(12):
                    j = LEG_MJC_TO_MPC[i]  
                    self.low_cmd.motor_cmd[i].q   = 2.146e9  
                    self.low_cmd.motor_cmd[i].kp  = 0.0
                    self.low_cmd.motor_cmd[i].dq  = 0.0
                    self.low_cmd.motor_cmd[i].kd  = 1.0      
                    self.low_cmd.motor_cmd[i].tau = legTorques[j]
            
            if self.debug_logger is not None:
                self._log_mpc_debug(robotRunner, running_time, commands, legTorques)

            time.sleep(dt)

            if Parameters.locomotionUnsafe:
                gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
                Parameters.locomotionUnsafe = False

        if use_gamepad:
            print("exiting MPC_RUN loop, stopping gamepad thread...")
            gamepad.stop()

    def _init_debug_loggers(self, log_dir):
        os.makedirs(log_dir, exist_ok=True)
        mpc_fields = (
            ["wall_time_ns", "running_time", "cmd_vx", "cmd_vy", "cmd_wz"]
            + vec_fields("low_q_FR", 3)
            + vec_fields("low_q_RL", 3)
            + vec_fields("low_dq_FR", 3)
            + vec_fields("low_dq_RL", 3)
            + vec_fields("tau_FR", 3)
            + vec_fields("tau_RL", 3)
            + vec_fields("imu_quat", 4)
            + vec_fields("imu_gyro", 3)
            + vec_fields("high_pos", 3)
            + vec_fields("high_vel", 3)
            + vec_fields("se_rpy", 3)
            + vec_fields("se_pos", 3)
            + vec_fields("se_vbody", 3)
            + vec_fields("se_omega_body", 3)
            + vec_fields("mpc_x", 12)
            + vec_fields("mpc_x_des", 12)
            + vec_fields("mpc_u_grf", 12)
            + vec_fields("foot_p_FR", 3)
            + vec_fields("foot_p_des_FR", 3)
            + vec_fields("foot_v_des_FR", 3)
            + vec_fields("foot_p_RL", 3)
            + vec_fields("foot_p_des_RL", 3)
            + vec_fields("foot_v_des_RL", 3)
        )
        pub_fields = (
            ["wall_time_ns", "pub_seq", "crc"]
            + vec_fields("tau_cmd_FR", 3)
            + vec_fields("tau_cmd_RL", 3)
            + vec_fields("q_cmd_FR", 3)
            + vec_fields("q_cmd_RL", 3)
        )
        self.debug_logger = CsvLogger(os.path.join(log_dir, "controller_mpc.csv"), mpc_fields)
        self.pub_logger = CsvLogger(os.path.join(log_dir, "controller_lowcmd_pub.csv"), pub_fields)
        print(f"[SDK2 Debug] writing controller logs to {log_dir}")

    def _log_mpc_debug(self, robotRunner, running_time, commands, legTorques):
        se = robotRunner._stateEstimator.getResult()
        cMPC = robotRunner._controlFSM.statesList.locomotion.cMPC
        snap = _get_mpc_snapshot(cMPC, se)
        row = {
            "wall_time_ns": wall_time_ns(),
            "running_time": running_time,
            "cmd_vx": float(commands[0]),
            "cmd_vy": float(commands[1]),
            "cmd_wz": float(commands[2]),
        }
        add_vec(row, "low_q_FR",  [self.low_state.motor_state[i].q  for i in range(3)],     3)
        add_vec(row, "low_q_RL",  [self.low_state.motor_state[i].q  for i in range(9, 12)], 3)
        add_vec(row, "low_dq_FR", [self.low_state.motor_state[i].dq for i in range(3)],     3)
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
        add_vec(row, "mpc_x",     snap.get("mpc_x",     []), 12)
        add_vec(row, "mpc_x_des", snap.get("mpc_x_des", []), 12)
        add_vec(row, "mpc_u_grf", snap.get("mpc_u_grf", []), 12)
        fp    = snap.get("foot_p",     [])
        fpdes = snap.get("foot_p_des", [])
        fvdes = snap.get("foot_v_des", [])
        add_vec(row, "foot_p_FR",     fp[3:6]    if len(fp)    >= 6 else [], 3)
        add_vec(row, "foot_p_RL",     fp[6:9]    if len(fp)    >= 9 else [], 3)
        add_vec(row, "foot_p_des_FR", fpdes[3:6] if len(fpdes) >= 6 else [], 3)
        add_vec(row, "foot_p_des_RL", fpdes[6:9] if len(fpdes) >= 9 else [], 3)
        add_vec(row, "foot_v_des_FR", fvdes[3:6] if len(fvdes) >= 6 else [], 3)
        add_vec(row, "foot_v_des_RL", fvdes[6:9] if len(fvdes) >= 9 else [], 3)
        self.debug_logger.write(row)

if __name__=="__main__":
    try:
        # Initialize DDS communication
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        mpc_locomotion = MPCLocomotionSDK2()
        mpc_locomotion.Init()
        mpc_locomotion.Start()
        mpc_locomotion.MPC_RUN()

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
        sys.exit(0)

    finally:
        if 'gamepad' in locals() and use_gamepad:
            print("exiting main, stopping gamepad thread...")
            gamepad.stop()
