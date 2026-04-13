import sys
import time
import numpy as np
from MPC_Controller.Parameters import Parameters
from MPC_Controller.robot_runner.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.utils import DTYPE
#from RL_Environment import gamepad_reader
import mujoco
from mujoco_sim.input_control import InputHandler #inpu control
from mujoco_sim.mujoco_sim_utils import *
from argparse import ArgumentParser
from MPC_Controller.utils import GaitType, FSM_StateName

parser = ArgumentParser(prog="RL_MPC_LOCOMOTION")
parser.add_argument("--render-fps", type=int, default=60, help="render fps")
# 移除了 num-envs，專注於模擬單一機器人
args = parser.parse_args()

#robot viewer
STAND_TARGET = np.array([
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763 
], dtype=DTYPE)

SIT_TARGET = np.array([
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375  
], dtype=DTYPE)

target = SIT_TARGET.copy()
is_started = False

KP_FRONT = 50.0
KD_FRONT = np.array([3.5, 0, 0, 0, 2.0, 0, 0, 0, 5.0], dtype=DTYPE).reshape((3,3))

KP_BACK = 85.0   
KD_BACK = np.array([5.0, 0, 0, 0, 4.0, 0, 0, 0, 5.0], dtype=DTYPE).reshape((3,3)) 
    
def main():
    # robot = RobotType[args.robot.upper()]
    # 暫時固定使用 go2 (fake AI)，因為它的 XML 已經包含了地形和機器人模型，方便測試
    dt = 0.002 #Parameters.controller_dt 
    model = mujoco.MjModel.from_xml_path("assets/go2/scene.xml")  # 直接載入 go2 的 xml，裡面已經包含了地形和機器人模型
    model.opt.timestep = dt
    data = mujoco.MjData(model)
    
    # Launch MuJoCo passive viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.cam.lookat = [0.0, 0.0, 0.0]
    viewer.cam.distance = 2.0
    
    # Set up MPC controller
    robotRunner = RobotRunnerFSM()
    robotRunner.init(RobotType.A1)
    
    count = 0
    render_fps = args.render_fps
    render_count = max(1, int(1 / render_fps / dt))

    # set up input handler
    input_handler = InputHandler()
    input_handler.start()

    runing_time, phase = 0.0, 0.0
    
    while viewer.is_running() and not input_handler.is_exit:
        step_start = time.time()
        runing_time += dt

        if is_started:
            
            # Calculate tanh
            if runing_time < 3.0:
                phase = np.tanh(runing_time / 1.2)  # Smooth 0→1 transition   
                if phase >= 0.99:  
                    phase = 1.0

                # current states
                q = data.sensordata[0:12]
                dq = data.sensordata[12:24]
                tau = np.zeros(12, dtype=DTYPE)
                
                for leg in range(4):
                    kp,kd_matrix = KP_FRONT, KD_FRONT if leg < 2 else KP_BACK, KD_BACK
                    kp_val = kp * phase + 20 * (1 - phase)
                    kp_matrix = kp_val * np.eye(3)
                    
                    q_target_leg = phase * STAND_TARGET[3*leg : 3*(leg+1)] + (1-phase) * SIT_TARGET[3*leg : 3*(leg+1)]

                    tau_leg = kp_matrix @ (q_target_leg - q[3*leg : 3*(leg+1)]) - kd_matrix @ dq[3*leg : 3*(leg+1)]
                    legTorques[3*leg : 3*(leg+1)] = tau_leg

            elif input_handler.is_moving:
                Parameters.cmpc_gait = GaitType.TROT
                Parameters.control_mode = FSM_StateName.LOCOMOTION

                commands = np.array([input_handler.vx, input_handler.vy, input_handler.angv], dtype=DTYPE)
                
                # run controllers
                dof_states = get_dof_state(model, data)  # get_actor_dof_states returns "pos","<f4" and "vel","<f4" in a structured array. <f4 means little-endian (stores data from LSB at smallest mm addr, and MSB at largest mm addr) single-precision float 32bit
                body_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base" )  #other robots: robotRunner._quadruped._bodyName
                body_states = get_body_state(data, body_idx) 
                legTorques = robotRunner.run(dof_states, body_states, commands).astype(np.float32)
            
            else:
               for leg in range(4):
                    kp, kd_matrix = KP_FRONT, KD_FRONT if leg < 2 else KP_BACK, KD_BACK
                    legTorques[3*leg : 3*(leg+1)] = kp * (STAND_TARGET[3*leg : 3*(leg+1)] - data.sensordata[3*leg : 3*(leg+1)]) - kd_matrix @ data.sensordata[12 + 3*leg : 12 + 3*(leg+1)] 

            data.ctrl[:] = legTorques

        if Parameters.locomotionUnsafe:
            # gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
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

    sys.exit(0)

if __name__=="__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
        sys.exit(0)