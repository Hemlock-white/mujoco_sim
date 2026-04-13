import numpy as np
import mujoco
from MPC_Controller.math_utils.orientation_tools import DTYPE
from mujoco import viewer
from MPC_Controller.utils import DTYPE


def get_dof_state(data):
    # dtype= dtype([('pos', '<f4'), ('vel', '<f4')])
    
    dof_state = np.dtype([
        ('pos', '<f4'), 
        ('vel', '<f4')
    ])
    Dof_state = np.zeros(1, dtype=dof_state)[0]
    for i in range(12):
        Dof_state[i]["pos"] = data.sensordata[i]
        Dof_state[i]["vel"] = data.sensordata[12 + i]

    return Dof_state
    """
    # 建立長度為 12 的陣列 (代表 12 個關節)
    dof_state = np.zeros(12, dtype=[('pos', '<f4'), ('vel', '<f4')])
    
    # 為了最安全起見 (不受 xml sensor 順序影響)，我們直接從物理引擎核心 qpos/qvel 抓取
    jnt_idx = 0
    for i in range(model.njnt):
        # 跳過軀幹的 freejoint (浮動基座)
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
            continue
            
        if jnt_idx < 12:
            qpos_adr = model.jnt_qposadr[i]
            qvel_adr = model.jnt_dofadr[i]
            dof_state["pos"][jnt_idx] = data.qpos[qpos_adr]
            dof_state["vel"][jnt_idx] = data.qvel[qvel_adr]
            jnt_idx += 1
    
    return dof_state"""

def get_body_state(data, body_id):
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
    
    w, x, y, z = data.xquat[body_id]
    Body_state['pose']['r'] = (x, y, z, w) 
    
    # use the following: world-aligned linear and angular velocity, and rotate it to body
    Body_state['vel']['linear'] = tuple(data.cvel[body_id][3:6])
    Body_state['vel']['angular'] = tuple(data.cvel[body_id][:3])
    
    return Body_state

"""
def standby(data, STAND_TARGET): 
    # think about move and stopped at a wierd pose, how do I smoothly transition to a safe 
    # standing pose instead of just commanding the stand target (which might cause high torque 
    # if the current pose is far from the stand target). So we can reuse the sit_stand_transition 
    # function with the current pose as the start and the stand target as the end, and maybe a 
    # faster transition time"""