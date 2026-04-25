import numpy as np
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