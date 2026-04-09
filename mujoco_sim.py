import math
import time
import numpy as np
from MPC_Controller.Parameters import Parameters
from MPC_Controller.robot_runner.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.robot_runner.RobotRunnerMin import RobotRunnerMin
from MPC_Controller.robot_runner.RobotRunnerPolicy import RobotRunnerPolicy
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.utils import DTYPE, ControllerType
from RL_Environment import gamepad_reader
import mujoco
from mujoco_sim.mujoco_sim_utils import *
from argparse import ArgumentParser
from MPC_Controller.utils import GaitType, FSM_OperatingMode, FSM_StateName
import threading

parser = ArgumentParser(prog="RL_MPC_LOCOMOTION")
# 暫時default use go2 scene.xml，因為它已經包含了地形和機器人模型，方便測試。你可以根據需要切換回 world.xml 或其他場景。
parser.add_argument("--robot", default="A1", choices=[name.title() for name in RobotType.__members__.keys()], help="robot types")
# parser.add_argument("--terrain", default="flat", choices=["flat", "slope", "stairs"], help="terrain types")
parser.add_argument("--mode", default="Fsm", choices=[name.title() for name in ControllerType.__members__.keys()], help="controller types")
parser.add_argument("--render-fps", type=int, default=60, help="render fps")
parser.add_argument("--disable-gamepad", action="store_true")
parser.add_argument("--checkpoint", default=None)
# 移除了 num-envs，專注於模擬單一機器人
args = parser.parse_args()


use_gamepad = not args.disable_gamepad
if use_gamepad:
    gamepad = gamepad_reader.Gamepad(vel_scale_x=2.5, vel_scale_y=1.5, vel_scale_rot=3.0)

#robot viewer
STAND_TARGET = np.array([
    0.0, 0.45, -0.9,  # FR hip>thigh>calf
    0.0, 0.45, -0.9,  # FL
    0.0, 0.45, -0.9,  # RR
    0.0, 0.45, -0.9   # RL
], dtype=DTYPE)

SIT_TARGET = np.array([
    0.0, 1.2, -2.5,  
    0.0, 1.2, -2.5,  
    0.0, 1.2, -2.5,  
    0.0, 1.2, -2.5   
], dtype=DTYPE)

target = SIT_TARGET.copy()
is_started = False

KP_stand = 50.0  # Higher stiffness for standing
KP_sit = 20.0    # Lower stiffness for sitting
KD = np.array([2, 0, 0, 0, 2, 0, 0, 0, 2], dtype=DTYPE).reshape((3,3))


def get_cmd():
    global target
    global is_started
    while True:
        if not is_started:
            start = input("Start? (y/n)")
            if start.lower() == 'y':
                is_started = True

        cmd = input("sit / stand / exit?")
        if cmd == "exit":
            target = None
        elif cmd == "sit":
            target = SIT_TARGET
        elif cmd == "stand":
            target = STAND_TARGET
        else:
            print("Invalid command. Please enter 'sit', 'stand', or 'exit'.")

def main():
    # for cmd
    if not use_gamepad:
        vel_x = 0.0
        vel_y = 0.0
        vel_rot = 0.0
        vel_scale_x = 2.5
        vel_scale_y = 1.5
        vel_scale_rot = 3.0
        is_e_stopped = True

    robot = RobotType[args.robot.upper()]
    # 暫時固定使用 go2 (fake AI)，因為它的 XML 已經包含了地形和機器人模型，方便測試
    dt = Parameters.controller_dt 
    model = mujoco.MjModel.from_xml_path("assets/go2/scene.xml")  # 直接載入 go2 的 xml，裡面已經包含了地形和機器人模型
    model.opt.timestep = dt
    data = mujoco.MjData(model)
    
    # Launch MuJoCo passive viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.cam.lookat = [0.0, 0.0, 0.0]
    viewer.cam.distance = 2.0

    # Set up MPC controller
    controller_type = ControllerType[args.mode.upper()]
    if controller_type is ControllerType.FSM:
        robotRunner = RobotRunnerFSM()
    elif controller_type is ControllerType.MIN:
        robotRunner = RobotRunnerMin()
    elif controller_type is ControllerType.POLICY:
        robotRunner = RobotRunnerPolicy(args.checkpoint)
    else:
        raise Exception("Invalid ControllerType!")

    #robotRunner.init(robot)

    count = 0
    render_fps = args.render_fps
    render_count = max(1, int(1 / render_fps / dt))

    input_thread = threading.Thread(target=get_cmd, daemon=True)
    input_thread.start()

    stand_time = 1.2  # seconds to complete standing motion
    transition_start_time = None
    current_target = SIT_TARGET.copy()

    # Main simulation loop
    while viewer.is_running():
        step_start = time.time()
        """
        # 1. 處理輸入指令
        if use_gamepad:
            lin_speed, ang_speed, e_stop = gamepad.get_command()
            Parameters.cmpc_gait = gamepad.get_gait()
            Parameters.control_mode = gamepad.get_mode()    
        else:
            lin_speed, ang_speed, e_stop = [vel_x*vel_scale_x, vel_y*vel_scale_y], vel_rot*vel_scale_rot, is_e_stopped  # 預設前進，無旋轉，非緊急停止
            Parameters.cmpc_gait = GaitType.TROT
            Parameters.control_mode = FSM_StateName.LOCOMOTION
        if not e_stop:
            commands = np.array([lin_speed[0], lin_speed[1], ang_speed], dtype=DTYPE)
        else:
            commands = np.zeros(3, dtype=DTYPE)
        
        # run controllers
        dof_states = get_dof_state(model, data)  # get_actor_dof_states returns "pos","<f4" and "vel","<f4" in a structured array. <f4 means little-endian (stores data from LSB at smallest mm addr, and MSB at largest mm addr) single-precision float 32bit
        body_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robotRunner._quadruped._bodyName)
        body_states = get_body_state(data, body_idx)  
        legTorques = robotRunner.run(dof_states, body_states, commands).astype(np.float32)
        """

        if is_started:
            if target is None:
                print("Exiting...")
                break

            # When target changes, start transition timer
            if not np.array_equal(target, current_target):
                transition_start_time = time.time()
                current_target = target.copy()
            
            # Calculate smooth phase using tanh
            if transition_start_time is not None:
                elapsed_time = time.time() - transition_start_time
                phase = np.tanh(elapsed_time / stand_time)  # Smooth 0→1 transition
                
                if phase >= 0.99:  # Transition complete
                    transition_start_time = None
                    phase = 1.0
            else:
                phase = 1.0

            # Get current state
            q = data.sensordata[0:12]
            dq = data.sensordata[12:24]
            tau = np.zeros(12, dtype=DTYPE)
            
            for leg in range(4):
                target_leg = current_target[3*leg : 3*(leg+1)]
                current_q_leg = q[3*leg : 3*(leg+1)]
                current_dq_leg = dq[3*leg : 3*(leg+1)]

                if target == STAND_TARGET:
                    kp_current = KP_stand * phase + KP_sit * (1 - phase)
                else:
                    kp_current = KP_stand
                kp_matrix = kp_current * np.eye(3)        
                
                smooth_target = phase * target_leg + (1-phase) * 

                tau_leg = kp_matrix @ (target_leg - current_q_leg) - KD @ current_dq_leg
                tau[3*leg : 3*(leg+1)] = tau_leg
            
            data.ctrl[:] = tau

        if Parameters.locomotionUnsafe:
            if use_gamepad:
                gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
            else:
                is_e_stopped = True
            Parameters.locomotionUnsafe = False
                
        # 5. 步進物理引擎
        mujoco.mj_step(model, data)

        # 6. 渲染畫面
        if count % render_count == 0:
            if model.nq >= 3:
                viewer.cam.lookat[0] = data.qpos[0]
                viewer.cam.lookat[1] = data.qpos[1]
            viewer.sync()

        # 7. 真實時間同步
        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
            
        count += 1

    if use_gamepad:
        gamepad.stop()

if __name__=="__main__":
    main()