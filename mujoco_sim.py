from pyexpat import model
import sys
import time
from xml.parsers.expat import model
import numpy as np
from MPC_Controller.Parameters import Parameters
from MPC_Controller.robot_runner.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.utils import DTYPE
from mujoco_sim import udp_reader
import mujoco
#from mujoco_sim.input_control import InputHandler #inpu control
from mujoco_sim.mujoco_sim_utils import *
from argparse import ArgumentParser
from MPC_Controller.utils import GaitType, FSM_StateName

parser = ArgumentParser(prog="RL_MPC_LOCOMOTION")
parser.add_argument("--disable-gamepad", action="store_true")
parser.add_argument("--render-fps", type=int, default=60, help="render fps")
# 移除了 num-envs，專注於模擬單一機器人
args = parser.parse_args()
use_gamepad = not args.disable_gamepad
if use_gamepad:
    gamepad = udp_reader.UDPGamepad(port=9876)

#robot viewer

    
def main():
    global target
    # robot = RobotType[args.robot.upper()]
    # 暫時固定使用 go2 (fake AI)，因為它的 XML 已經包含了地形和機器人模型，方便測試
    dt = 0.005 # Match Isaac Gym physics step (0.01 / 2 substeps)
    model = mujoco.MjModel.from_xml_path("assets/go2/scene.xml")  # 直接載入 go2 的 xml，裡面已經包含了地形和機器人模型
    model.opt.timestep = dt
    data = mujoco.MjData(model)
    
    # Launch MuJoCo passive viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.cam.lookat = [0.0, 0.0, 0.0]
    viewer.cam.distance = 2.0
    
    # Set up MPC controller
    robotRunner = RobotRunnerFSM()
    robotRunner.init(RobotType.GO2)
    #print(f"robot mass: {robotRunner._quadruped._bodyMass}")
    
    count = 0
    render_fps = args.render_fps
    render_count = max(1, int(1 / render_fps / dt))

    # set up input handler
    #input_handler = InputHandler()
    #input_handler.start()

    
    running_time = 0.0
    legTorques = np.zeros(12, dtype=DTYPE)
    """ 初始化 CSV Logger (在 Isaac Gym 那邊可以改成 "isaac_log.csv")"""
    log_file = init_csv_logger("mujoco_log.csv")
    
    while viewer.is_running(): #and not input_handler.is_exit
        step_start = time.time()
        running_time += dt
        commands = np.zeros(3, dtype=DTYPE)          
        
        if use_gamepad:
            if gamepad.is_standing:
                # When target changes, start transition timer
                legTorques = pd_stand(data, running_time)
            
            if gamepad.is_moving:
                        
                lin_speed, ang_speed, e_stop = gamepad.get_command()
                Parameters.cmpc_gait = gamepad.get_gait()
                Parameters.control_mode = gamepad.get_mode()
                if not e_stop:
                    commands = np.array([lin_speed[0], lin_speed[1], ang_speed], dtype=DTYPE)
                
                # run controllers
                dof_states = get_dof_state(data)  # get_actor_dof_states returns "pos","<f4" and "vel","<f4" in a structured array. <f4 means little-endian (stores data from LSB at smallest mm addr, and MSB at largest mm addr) single-precision float 32bit
                body_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link" )  #other robots: robotRunner._quadruped._bodyName
                body_states = get_body_state(model, data, body_idx) 
                legTorques = robotRunner.run(dof_states, body_states, commands).astype(np.float32)
                
                """ ====== 新增：呼叫 Logging 函式 ======"""
                #raw_pos_z = body_states['pose']['p'][2] # 抓取實際高度 (MuJoCo 格式)
                body_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "FL_foot" )  #other robots: robotRunner._quadruped._bodyName
                body_states = get_body_state(model, data, body_idx) 
                fl_foot_z = body_states['pose']['p'][2]
                # 如果是在 Isaac Gym 裡，可能是 raw_pos_z = body_states["pose"]["p"][0][2]
                log_mpc_states(log_file, data.time, robotRunner, fl_foot_z, legTorques)
            data.ctrl[:] = legTorques
        
        if Parameters.locomotionUnsafe:
            gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
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

    """ 當離開 while 迴圈後"""
    if log_file is not None:
        log_file.flush()  
        log_file.close()
        print("\n[Logger] CSV 檔案已成功儲存！")

    if use_gamepad:
        gamepad.stop()

    sys.exit(0)

if __name__=="__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
        sys.exit(0)
