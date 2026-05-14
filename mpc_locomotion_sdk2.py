"""MPC Locomotion related headers and functions"""
import sys
import time
import numpy as np
from MPC_Controller.Parameters import Parameters
from MPC_Controller.robot_runner.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.utils import DTYPE
from mujoco_sim import udp_reader
#import mujoco
from mujoco_sim.mujoco_sim_utils import *
from argparse import ArgumentParser

"""sdk related libraries"""
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from mujoco_sim import config_sdk2 as config

""""""
parser = ArgumentParser(prog="mpc_locomotion_sdk2")
parser.add_argument("--disable-gamepad", action="store_true")
parser.add_argument("--enable-motion-switcher", action="store_true")
# 移除了 num-envs，專注於模擬單一機器人
args = parser.parse_args()
use_gamepad = not args.disable_gamepad
if use_gamepad:
    gamepad = udp_reader.UDPGamepad(port=9876)



class MPCLocomotionSDK2:
    def __init__(self):
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None
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
        """
        if args.enable_motion_switcher:
            self.msc = MotionSwitcherClient()
            self.msc.SetTimeout(5.0)  # Optional: set RPC timeout
            self.msc.Init()            # Register APIs and set version

            status, result = self.msc.CheckMode()
            while result['name']:
                self.msc.StandDown()
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)"""

    def Start(self): # the thread that 
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=0.005, target=self.LowCmdWrite, name="writebasiccmd"
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
        # print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
        # print("IMU state: ", msg.imu_state)
        # print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)

    def LowCmdWrite(self):
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)
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
        waiting_for_low_state = True
        
        while True:
            commands = np.zeros(3, dtype=DTYPE)          
            running_time += 0.005

            if not use_gamepad:
                break

            if self.low_state is None:
                if waiting_for_low_state:
                    print("Waiting for rt/lowstate from mujoco_sim_sdk2.py...")
                    waiting_for_low_state = False
                time.sleep(0.005)
                continue

            if gamepad.is_standing:
            # When target changes, start transition timer
                legTorques = pd_stand_sdk2(self.low_state, running_time)
                
            if gamepad.is_moving:
                lin_speed, ang_speed, e_stop = gamepad.get_command()
                Parameters.cmpc_gait = gamepad.get_gait()
                Parameters.control_mode = gamepad.get_mode()
                if not e_stop:
                    commands = np.array([lin_speed[0], lin_speed[1], ang_speed], dtype=DTYPE)
                    
                # run controllers
                dof_states = get_dof_state_sdk2(self.low_state)  # get_actor_dof_states returns "pos","<f4" and "vel","<f4" in a structured array. <f4 means little-endian (stores data from LSB at smallest mm addr, and MSB at largest mm addr) single-precision float 32bit
                body_states = get_body_state_sdk2(self.low_state)
                # FL -> FR -> RL -> RR but sdk in R -> L -> R -> L
                legTorques = robotRunner.run(dof_states, body_states, commands).astype(np.float32)
                
            """       
            for i in range(12):# R and L swap: 0-2 <-> 3-5, 6-8 <-> 9-11
                if (i%6 <= 2):
                    self.low_cmd.motor_cmd[i+3].tau = legTorques[i]  
                else:
                    self.low_cmd.motor_cmd[i-3].tau = legTorques[i]"""
            print("legTorques: ", legTorques)
            for i in range(12):
                self.low_cmd.motor_cmd[i].tau = legTorques[i]

            time.sleep(0.005)

            if Parameters.locomotionUnsafe:
                gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
                Parameters.locomotionUnsafe = False

        if use_gamepad:
            print("exiting MPC_RUN loop, stopping gamepad thread...")
            gamepad.stop()

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
