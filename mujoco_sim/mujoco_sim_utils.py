import numpy as np
import mujoco
import mujoco.viewer

# Asset paths (指向你創建的 world.xml 或機器人的 URDF)
ASSET_ROOT = "assets"
ROBOT_URDF = {
    "ALIENGO": "aliengo_description/urdf/aliengo.urdf",
    "A1": "a1_description/urdf/a1.urdf",
    "GO1": "go1_description/urdf/go1.urdf"
}

# Simulation parameters
init_height = 0.5

def load_model(robot_type):
    """
    Load MuJoCo model for the given RobotType enum or string.
    Accepts RobotType enum or string (e.g., 'ALIENGO').
    """
    try:
        urdf_path = ROBOT_URDF.get(robot_type)
        if urdf_path is None:
            raise Exception(f"Unknown robot type: {robot_type}")
        model = mujoco.MjModel.from_xml_path(f"{ASSET_ROOT}/{urdf_path}")
        return model
    except Exception as e:
        print(f"Error loading URDF/XML: {e}")
        raise

def get_dof_states(model, data):
    """
    獲取關節狀態，並打包成 Isaac Gym 的結構化 Numpy 陣列格式
    dtype=[('pos', 'f4'), ('vel', 'f4')]
    """
    num_dofs = model.nu # 假設致動器數量等於受控關節數量 (通常是12)
    dof_states = np.zeros(num_dofs, dtype=[('pos', 'f4'), ('vel', 'f4')])
    
    for i in range(num_dofs):
        # 找到致動器對應的關節
        joint_id = model.actuator_trnid[i, 0]
        qpos_idx = model.jnt_qposadr[joint_id]
        qvel_idx = model.jnt_dofadr[joint_id]
        
        dof_states['pos'][i] = data.qpos[qpos_idx]
        dof_states['vel'][i] = data.qvel[qvel_idx]
        
    return dof_states

def get_body_state(model, data, body_name):
    """
    獲取 Base Link 狀態，並打包成 Isaac Gym 的結構化格式
    IsaacGym Quaternion 是 [x, y, z, w], 而 MuJoCo 是 [w, x, y, z]
    """
    # 建立與 Isaac Gym 相同的資料結構
    body_state = np.zeros(1, dtype=[
        ('pose', [('p', 'f4', (3,)), ('r', 'f4', (4,))]),
        ('vel', [('linear', 'f4', (3,)), ('angular', 'f4', (3,))])
    ])[0] # 提取裡面的單一 record
    
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        print(f"Warning: Body '{body_name}' not found! Using root.")
        body_id = 1 # 通常 1 是 base_link
        
    # 位置 (Position)
    body_state['pose']['p'] = data.xpos[body_id]
    
    # 姿態 (Quaternion): MuJoCo (w,x,y,z) -> IsaacGym (x,y,z,w)
    wq, xq, yq, zq = data.xquat[body_id]
    body_state['pose']['r'] = [xq, yq, zq, wq]
    
    # 速度 (Velocity): 獲取世界座標系下的線速度與角速度
    vel = np.zeros(6)
    # 0 代表獲取 global frame 的速度
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, vel, 0) 
    
    # mj_objectVelocity 的回傳格式是 [角速度, 線速度]
    body_state['vel']['angular'] = vel[0:3]
    body_state['vel']['linear'] = vel[3:6]
    
    return body_state

def reset_robot(model, data):
    """重置機器人到初始高度與姿態"""
    mujoco.mj_resetData(model, data)
    
    # 假設 base_link 使用 free joint，會佔據 qpos 的前 7 個位置 (3 pos + 4 quat)
    if model.nq >= 7:
        data.qpos[0:3] = [0.0, 0.0, init_height]
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0] # MuJoCo format (w, x, y, z)
    mujoco.mj_forward(model, data)