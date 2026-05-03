import numpy as np
import os
import mujoco
from MPC_Controller.math_utils.orientation_tools import DTYPE
from mujoco import viewer
from MPC_Controller.utils import DTYPE

STAND_TARGET = np.array([
    0., 0.8, -1.6,
    0., 0.8, -1.6,
    0., 0.8, -1.6,
    0., 0.8, -1.6
], dtype=DTYPE)

SIT_TARGET = np.array([
    0.0, 1.4, -2.7,
    -0.0, 1.4, -2.7,
    0.0, 1.4, -2.7,
    -0.0, 1.4, -2.7  
], dtype=DTYPE)

target = STAND_TARGET.copy()

KP_FRONT = 50.0
KD_FRONT = np.array([5, 0, 0, 0, 5.0, 0, 0, 0, 5.0], dtype=DTYPE).reshape((3,3))

KP_BACK = 85.0   
KD_BACK = np.array([7.0, 0, 0, 0, 7.0, 0, 0, 0, 7.0], dtype=DTYPE).reshape((3,3)) 

transition_start_time = None
last_target = np.zeros(12, dtype=DTYPE)
q_start = np.zeros(12, dtype=DTYPE)
LegTorques = np.zeros(12, dtype=DTYPE)

def get_dof_state(data):
    # dtype= dtype([('pos', '<f4'), ('vel', '<f4')])
    
    dof_state = np.dtype([
        ('pos', '<f4'), 
        ('vel', '<f4')
    ])
    Dof_state = np.zeros(12, dtype=dof_state)
    Dof_state["pos"] = data.sensordata[0:12] 
    Dof_state["vel"] = data.sensordata[12:24]

    return Dof_state

def get_body_state(model, data, body_id):
    # dtype= dtype([('pose', [('p', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]), 
    #                         ('r', [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('w', '<f4')])]), 
    #               ('vel', [('linear', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]), 
    #                        ('angular', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')])])])
    body_state = np.dtype([
        ('pose', [
            ('p', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]),
            ('r', [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('w', '<f4')])
        ]),
        ('vel', [
            ('linear', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]),
            ('angular', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        ])
    ])
    Body_state = np.zeros(1, dtype=body_state)[0] #好像初始化一個是可以的?
    Body_state['pose']['p'] = tuple(data.xpos[body_id])
    
    w, x, y, z = data.xquat[body_id] #w, x, y, z = data.qpos[3:7]
    Body_state['pose']['r'] = (x, y, z, w) 
    
    body_vel = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, body_vel, 0)
    Body_state['vel']['linear'] = tuple(body_vel[3:6])
    Body_state['vel']['angular'] = tuple(body_vel[:3])
    
    return Body_state

def get_dof_state_sdk2(low_state):
    dof_state = np.dtype([
        ('pos', '<f4'), 
        ('vel', '<f4')
    ])
    Dof_state = np.zeros(12, dtype=dof_state)
    for i in range(12):
        if (i%6 <= 2): # R and L swap: 0-2 <-> 3-5, 6-8 <-> 9-11
            Dof_state["pos"][i+3] = low_state.motor_state[i].q
            Dof_state["vel"][i+3] = low_state.motor_state[i].dq
        else:
            Dof_state["pos"][i-3] = low_state.motor_state[i].q
            Dof_state["vel"][i-3] = low_state.motor_state[i].dq

    return Dof_state

def get_body_state_sdk2(low_state):
    body_state = np.dtype([
        ('pose', [
            ('p', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]),
            ('r', [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('w', '<f4')])
        ]),
        ('vel', [
            ('linear', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]),
            ('angular', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        ])
    ])
    Body_state = np.zeros(1, dtype=body_state)[0]
    Body_state['pose']['p'] = tuple(low_state.imu_state.rpy) 
    w, x, y, z = low_state.imu_state.quaternion
    Body_state['pose']['r'] = (x, y, z, w) 
    Body_state['vel']['linear'] = tuple(low_state.imu_state.accelerometer)
    Body_state['vel']['angular'] = tuple(low_state.imu_state.gyroscope)
    
    return Body_state

def pd_stand_sdk2(low_state, running_time):
    global last_target, transition_start_time, target, q_start, KD_BACK, KD_FRONT, KP_BACK, KP_FRONT
    if not np.array_equal(target, last_target):
        transition_start_time = running_time
        q_start = np.zeros(12, dtype=DTYPE)
        for i in range(12):
            if (i%6 <= 2): # R and L swap: 0-2 <-> 3-5, 6-8 <-> 9-11
                q_start[i+3] = low_state.motor_state[i].q
            else:
                q_start[i-3] = low_state.motor_state[i].q
        last_target = target.copy()
    
    # Calculate tanh
    if transition_start_time is not None:
        elapsed_time = running_time - transition_start_time
        phase = np.tanh(elapsed_time / 1.2)  # Smooth 0→1 transition   
        if phase >= 0.99:  
            phase = 1.0
    else:
        phase = 0.0
    
    # current states
    q = np.zeros(12, dtype=DTYPE)
    dq = np.zeros(12, dtype=DTYPE)
    for i in range(12):
        if (i%6 <= 2): # R and L swap: 0-2 <-> 3-5, 6-8 <-> 9-11
            q[i+3] = low_state.motor_state[i].q
            dq[i+3] = low_state.motor_state[i].dq
        else:
            q[i-3] = low_state.motor_state[i].q
            dq[i-3] = low_state.motor_state[i].dq

    for leg in range(4):
        target_leg = target[3*leg : 3*(leg+1)]
        q_start_leg = q_start[3*leg : 3*(leg+1)]
        current_q_leg = q[3*leg : 3*(leg+1)]
        current_dq_leg = dq[3*leg : 3*(leg+1)]

        # F/R leg pd control
        kp = KP_FRONT if leg < 2 else KP_BACK
        kd_matrix = KD_FRONT if leg < 2 else KD_BACK
        kp_val = kp * phase + 20 * (1 - phase)
        kp_matrix = kp_val * np.eye(3)

        smooth_tleg = phase * target_leg + (1-phase) * q_start_leg

        tau_leg = kp_matrix @ (smooth_tleg - current_q_leg) - kd_matrix @ current_dq_leg
        LegTorques[3*leg : 3*(leg+1)] = tau_leg
        
    return LegTorques

def pd_stand(data, running_time):
    global last_target, transition_start_time, target, q_start, KD_BACK, KD_FRONT, KP_BACK, KP_FRONT
    if not np.array_equal(target, last_target):
        transition_start_time = running_time
        q_start = data.sensordata[0:12]
        last_target = target.copy()
    
    # Calculate tanh
    if transition_start_time is not None:
        elapsed_time = running_time - transition_start_time
        phase = np.tanh(elapsed_time / 1.2)  # Smooth 0→1 transition   
        if phase >= 0.99:  
            phase = 1.0
    else:
        phase = 0.0
    
    # current states
    q = data.sensordata[0:12]
    dq = data.sensordata[12:24]

    for leg in range(4):
        target_leg = target[3*leg : 3*(leg+1)]
        q_start_leg = q_start[3*leg : 3*(leg+1)]
        current_q_leg = q[3*leg : 3*(leg+1)]
        current_dq_leg = dq[3*leg : 3*(leg+1)]

        # F/R leg pd control
        kp = KP_FRONT if leg < 2 else KP_BACK
        kd_matrix = KD_FRONT if leg < 2 else KD_BACK
        kp_val = kp * phase + 20 * (1 - phase)
        kp_matrix = kp_val * np.eye(3)
            
        smooth_tleg = phase * target_leg + (1-phase) * q_start_leg

        tau_leg = kp_matrix @ (smooth_tleg - current_q_leg) - kd_matrix @ current_dq_leg
        LegTorques[3*leg : 3*(leg+1)] = tau_leg
        
    return LegTorques

def init_csv_logger(filename="mujoco_log.csv"):
    """
    初始化 CSV 檔案並寫入標頭 (Header)。
    回傳一個開啟的檔案物件，供主迴圈寫入使用。
    """
    try:
        log_file = open(filename, "w")
        log_file.write("Time,Raw_PosZ,SE_Roll,SE_Pitch,vBody_X,FF_ForceZ_FR,FF_ForceX_FR,Tau_Hip_FR\n")
        return log_file
    except Exception as e:
        print(f"[Logger] 無法建立檔案: {e}")
        return None

def log_mpc_states(log_file, current_time, robot_runner, raw_pos_z, leg_torques):
    if log_file is None:
        return
        
    try:
        # 從大腦抓取資料
        se_result = robot_runner._stateEstimator.result
        leg_cmds = robot_runner._legController.commands
        
        # 【安全抓取法】：使用 .flatten() 避免 numpy 維度錯誤 (例如 [0][0] 變 [0])
        se_roll = se_result.rpy.flatten()[0]
        se_pitch = se_result.rpy.flatten()[1]
        vb_x = se_result.vBody.flatten()[0]
        
        # 抓取右前腳 (FR, index 0)
        ff_z = leg_cmds[0].forceFeedForward.flatten()[2]
        ff_x = leg_cmds[0].forceFeedForward.flatten()[0]
        
        # 確保 leg_torques 已經是正確格式
        tau_hip = float(leg_torques[0])
        raw_z_val = float(raw_pos_z) # 強制轉為純浮點數
        
        # 寫入資料並立刻存檔
        log_file.write(f"{current_time:.4f},{raw_z_val:.4f},{se_roll:.4f},{se_pitch:.4f},{vb_x:.4f},{ff_z:.4f},{ff_x:.4f},{tau_hip:.4f}\n")
        log_file.flush() # 強制立刻寫入硬碟！
        
    except Exception as e:
        # 把錯誤印出來我們才知道發生了什麼事！
        print(f"\r[Logger 報錯] {type(e).__name__}: {e}        ", end="")