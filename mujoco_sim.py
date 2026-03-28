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

parser = ArgumentParser(prog="RL_MPC_LOCOMOTION")
parser.add_argument("--robot", default="Aliengo", choices=[name.title() for name in RobotType.__members__.keys()], help="robot types")
parser.add_argument("--mode", default="Fsm", choices=[name.title() for name in ControllerType.__members__.keys()], help="controller types")
parser.add_argument("--render-fps", type=int, default=60, help="render fps")
parser.add_argument("--disable-gamepad", action="store_true")
parser.add_argument("--checkpoint", default=None)
# 移除了 num-envs，因為 MuJoCo 中我們專注於模擬單一 MPC 機器人
args = parser.parse_args()

use_gamepad = not args.disable_gamepad

if use_gamepad:
    gamepad = gamepad_reader.Gamepad(vel_scale_x=2.5, vel_scale_y=1.5, vel_scale_rot=3.0)

def main():
    robot = RobotType[args.robot.upper()]
    dt = Parameters.controller_dt
    
    # Load MuJoCo model for the selected robot
    model = load_model(robot)
    model.opt.timestep = dt
    data = mujoco.MjData(model)

    # Reset robot to initial pose
    reset_robot(model, data)

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

    robotRunner.init(robot)

    count = 0
    render_fps = args.render_fps
    render_count = max(1, int(1 / render_fps / dt))

    # Main simulation loop
    while viewer.is_running():
        step_start = time.time()

        # Step physics
        mujoco.mj_step(model, data)

        commands = np.zeros(3, dtype=DTYPE)
        if use_gamepad:
            lin_speed, ang_speed, e_stop = gamepad.get_command()
            Parameters.cmpc_gait = gamepad.get_gait()
            Parameters.control_mode = gamepad.get_mode()
            if not e_stop:
                commands = np.array([lin_speed[0], lin_speed[1], ang_speed], dtype=DTYPE)

        # Get states in MPC format
        dof_states = get_dof_states(model, data)
        # Use the correct body name from the Quadruped instance
        body_state = get_body_state(model, data, robotRunner._quadruped._bodyName)
        
        # 執行 MPC 取得關節力矩
        torques = robotRunner.run(dof_states, body_state, commands).astype(np.float32)

        # 應用力矩到致動器上
        data.ctrl[:] = torques         

        if Parameters.locomotionUnsafe:
            gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
            Parameters.locomotionUnsafe = False

        # 渲染畫面
        if count % render_count == 0:
            # 你可以讓相機跟隨機器人
            if model.nq >= 3:
                viewer.cam.lookat[0] = data.qpos[0]
                viewer.cam.lookat[1] = data.qpos[1]
            viewer.sync()

        # 真實時間同步 (取代 gym.sync_frame_time)
        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
            
        count += 1

    if use_gamepad:
        gamepad.stop()

if __name__=="__main__":
    main()