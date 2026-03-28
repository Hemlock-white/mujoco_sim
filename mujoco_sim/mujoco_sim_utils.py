import numpy as np
import mujoco
import mujoco.viewer
from MPC_Controller.common.Quadruped import RobotType

# Asset paths (指向你創建的 world.xml 或機器人的 URDF)
ASSET_ROOT = "assets"
ALIENGO = "aliengo_description/urdf/aliengo.urdf"
A1 = "a1_description/urdf/a1.urdf"
GO1 = "go1_description/urdf/go1.urdf"

# Simulation parameters
init_height = 0.5

def load_model(robot_type):
    """
    Load MuJoCo model for the given RobotType enum or string.
    Accepts RobotType enum or string (e.g., 'ALIENGO').
    """
    if robot_type is RobotType.ALIENGO:
        path = ALIENGO
    elif robot_type is RobotType.A1:
        path = A1
    elif robot_type is RobotType.GO1:
        path = GO1
    else:
        raise Exception("Invalid RobotType")
    try:
        # 如果你已經建好了 world.xml，可以將字串換成你的 xml 檔名
        model = mujoco.MjModel.from_xml_path(f"{ASSET_ROOT}/{path}")
        return model
    except Exception as e:
        print(f"Error loading URDF/XML: {e}")
        raise

def get_dof_states(model, data):
    """
    獲取關節狀態。
    修改：直接讀取關節(Joint)而非致動器(Actuator)，避開 URDF 沒有定義 actuator 的問題。
    """
    dof_pos = []
    dof_vel = []
    
    for i in range(model.njnt):
        # 跳過浮動基座的自由關節 (Free joint, 通常是機器人的身體)
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
            continue
            
        qpos_idx = model.jnt_qposadr[i]
        qvel_idx = model.jnt_dofadr[i]
        
        dof_pos.append(data.qpos[qpos_idx])
        dof_vel.append(data.qvel[qvel_idx])
        
    dof_states = np.zeros(len(dof_pos), dtype=[('pos', 'f4'), ('vel', 'f4')])
    dof_states['pos'] = dof_pos
    dof_states['vel'] = dof_vel
    
    return dof_states

def get_body_state(model, data, body_name):
    """
    獲取 Base Link 狀態，加入對名稱差異的容錯處理。
    """
    body_state = np.zeros(1, dtype=[
        ('pose', [('p', 'f4', (3,)), ('r', 'f4', (4,))]),
        ('vel', [('linear', 'f4', (3,)), ('angular', 'f4', (3,))])
    ])[0] 
    
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    
    # 增加容錯：如果找不到指定的 body_name (如 trunk)，試著找 base 或 base_link
    if body_id < 0:
        for fallback_name in ['trunk', 'base', 'base_link']:
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fallback_name)
            if body_id >= 0:
                break
                
    if body_id < 0:
        print(f"Warning: Body '{body_name}' not found! Using root (index 1).")
        body_id = 1 
        
    body_state['pose']['p'] = data.xpos[body_id]
    
    wq, xq, yq, zq = data.xquat[body_id]
    body_state['pose']['r'] = [xq, yq, zq, wq]
    
    vel = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, vel, 0) 
    
    body_state['vel']['angular'] = vel[0:3]
    body_state['vel']['linear'] = vel[3:6]
    
    return body_state

def apply_torques(model, data, torques):
    """
    將算出的力矩直接加在對應的關節自由度上，
    這樣 URDF 就算沒有 <actuator> 標籤也能運作。
    """
    torque_idx = 0
    for i in range(model.njnt):
        # 跳過浮動基座的自由關節
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
            continue
            
        if torque_idx < len(torques):
            dof_idx = model.jnt_dofadr[i]
            # qfrc_applied 允許我們直接施加外力/力矩
            data.qfrc_applied[dof_idx] = torques[torque_idx]
            torque_idx += 1
def reset_robot(model, data):
    """重置機器人到初始高度與姿態"""
    mujoco.mj_resetData(model, data)
    
    # 假設 base_link 使用 free joint，會佔據 qpos 的前 7 個位置 (3 pos + 4 quat)
    if model.nq >= 7:
        data.qpos[0:3] = [0.0, 0.0, init_height]
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0] # MuJoCo format (w, x, y, z)
    mujoco.mj_forward(model, data)