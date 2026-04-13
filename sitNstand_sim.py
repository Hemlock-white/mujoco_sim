import time
import numpy as np
from MPC_Controller.utils import DTYPE
#from RL_Environment import gamepad_reader
import mujoco
from mujoco_sim.mujoco_sim_utils import *
from argparse import ArgumentParser
import threading

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

target = STAND_TARGET.copy()
is_started = False

KP_FRONT_BASE = 50.0
KD_FRONT = np.array([3.5, 0, 0, 0, 2.0, 0, 0, 0, 5.0], dtype=DTYPE).reshape((3,3))

KP_BACK_BASE = 85.0   # 比前腿大
KD_BACK = np.array([5.0, 0, 0, 0, 4.0, 0, 0, 0, 5.0], dtype=DTYPE).reshape((3,3))

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
    # 暫時固定使用 go2 (fake AI)，因為它的 XML 已經包含了地形和機器人模型，方便測試
    dt = 0.002 
    model = mujoco.MjModel.from_xml_path("assets/go2/scene.xml")  # 直接載入 go2 的 xml，裡面已經包含了地形和機器人模型
    model.opt.timestep = dt
    data = mujoco.MjData(model)
    
    # Launch MuJoCo passive viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.cam.lookat = [0.0, 0.0, 0.0]
    viewer.cam.distance = 2.0

    count = 0
    render_fps = args.render_fps
    render_count = max(1, int(1 / render_fps / dt))

    input_thread = threading.Thread(target=get_cmd, daemon=True)
    input_thread.start()

    stand_time = 1.2  # seconds to complete standing motion
    transition_start_time = None
    last_target = np.zeros(12, dtype=DTYPE)
    runing_time = 0.0
    
    # Main simulation loop
    while viewer.is_running():
        step_start = time.time()
        runing_time += dt

        if is_started:
            if target is None:
                print("Exiting...")
                break

            # When target changes, start transition timer
            if not np.array_equal(target, last_target):
                transition_start_time = runing_time
                q_at_transiton_start = data.sensordata[0:12]
                last_target = target.copy()
            
            # Calculate tanh
            if transition_start_time is not None:
                elapsed_time = runing_time - transition_start_time
                phase = np.tanh(elapsed_time / stand_time)  # Smooth 0→1 transition   
                if phase >= 0.99:  
                    phase = 1.0
            else:
                phase = 0.0

            # current states
            q = data.sensordata[0:12]
            dq = data.sensordata[12:24]
            tau = np.zeros(12, dtype=DTYPE)
            
            for leg in range(4):
                target_leg = target[3*leg : 3*(leg+1)]
                q_start_leg = q_at_transiton_start[3*leg : 3*(leg+1)]
                current_q_leg = q[3*leg : 3*(leg+1)]
                current_dq_leg = dq[3*leg : 3*(leg+1)]

                if leg < 2: # 前腿
                    if np.array_equal(target, STAND_TARGET):
                        kp_val = KP_FRONT_BASE * phase + 20 * (1 - phase)
                    else:
                        kp_val = KP_FRONT_BASE
                    kp_matrix = kp_val * np.eye(3)
                    kd_matrix = KD_FRONT
                else: # 後腿
                    if np.array_equal(target, STAND_TARGET):
                        # 後腿起跳的力量可以維持在較高水平
                        kp_val = KP_BACK_BASE * phase + 30 * (1 - phase)
                    else:
                        kp_val = KP_BACK_BASE
                    kp_matrix = kp_val * np.eye(3)
                    kd_matrix = KD_BACK       
                
                smooth_tleg = phase * target_leg + (1-phase) * q_start_leg

                tau_leg = kp_matrix @ (smooth_tleg - current_q_leg) - kd_matrix @ current_dq_leg
                tau[3*leg : 3*(leg+1)] = tau_leg

            data.ctrl[:] = tau
              
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

    #if use_gamepad:
    #    gamepad.stop()

if __name__=="__main__":
    main()