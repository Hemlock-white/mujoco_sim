import os
import numpy as np
import mujoco
import mujoco.viewer
from MPC_Controller.common.Quadruped import RobotType

# Asset paths (指向你創建的 world.xml 或機器人的 URDF)
ASSET_ROOT = "assets"
ALIENGO = "aliengo_description/urdf/aliengo.urdf"
# A1 = "a1_description/urdf/a1.urdf"
# go2 in desguise of A1
A1 = "go2/go2.xml"
GO1 = "go1_description/urdf/go1.urdf"

# Simulation parameters
init_height = 0.5
""" default go2, so these are not used
def get_robot_path(robot_type): 
    if robot_type is RobotType.ALIENGO:
        path = ASSET_ROOT + "/" + ALIENGO
    elif robot_type is RobotType.A1:
        path = ASSET_ROOT + "/" + A1
    elif robot_type is RobotType.GO1:
        path = ASSET_ROOT + "/" + GO1
    else:
        raise Exception("Invalid RobotType")
    return path

def load_model(robot_type):
    path = get_robot_path(robot_type)
    try:
        model = mujoco.MjModel.from_xml_path(f"{path}")
        return model
    except Exception as e:
        print(f"Error loading URDF/XML: {e}")
        raise
"""
def get_dof_state(model, data):
    """
    dtype= dtype([('pos', '<f4'), ('vel', '<f4')])
    
    dof_state = np.dtype([
        ('pos', '<f4'), 
        ('vel', '<f4')
    ])
    Dof_state = np.zeros((), dtype=dof_state)
    Dof_state["pos"] = data.sensordata[:12] #avlueError: setting an array element with a sequence.
    Dof_state["vel"] = data.sensordata[12:24]

    return Dof_state"""
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

    return dof_state

def get_body_state(data, body_id):
    """
    # dtype= dtype([('pose', [('p', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]), 
    #                         ('r', [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('w', '<f4')])]), 
    #               ('vel', [('linear', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]), 
    #                        ('angular', [('x', '<f4'), ('y', '<f4'), ('z', '<f4')])])])
    """
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
    
    Body_state['vel']['linear'] = tuple(data.cvel[body_id][3:6])
    Body_state['vel']['angular'] = tuple(data.cvel[body_id][:3])
    
    return Body_state
