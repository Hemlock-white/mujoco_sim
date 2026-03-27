import numpy as np
import mujoco
import mujoco.viewer
from MPC_Controller.common.Quadruped import RobotType

# Asset paths
ASSET_ROOT = "assets"
ALIENGO = "aliengo_description/urdf/aliengo.urdf"
A1 = "a1_description/urdf/a1.urdf"
GO1 = "go1_description/urdf/go1.urdf"

# Foot link indices (these may need adjustment for mujoco)
FOOT_IDX = [4, 8, 12, 16]

# Simulation parameters
fix_base_link = False
init_height = 0.5

class MujocoSimulator:
    """Wrapper for Mujoco simulation to provide IsaacGym-like interface"""
    
    def __init__(self, dt=0.001, gravity=-9.81):
        self.dt = dt
        self.gravity = gravity
        self.model = None
        self.data = None
        self.viewer = None
        self.envs = []  # Store models for each env
        self.actor_handles = []  # Store data for each env
    def step(self):
        for model, data in self.actor_handles:
            mujoco.mj_step(model, data)

    def get_state(self):
        states = []
        for model, data in self.actor_handles:
            state = {
                "qpos": data.qpos.copy(),
                "qvel": data.qvel.copy()
            }
            states.append(state)
        return states
    
    def get_body_state(self, body_name):
        states = []
        for model, data in self.actor_handles:
            body_states = {
                    "pose": {
                        "p": data.xpos[0].copy(),
                        "r": data.xquat[0].copy()
                    },
                    "vel": {
                        "linear": data.cvel[0, 3:6].copy(),
                        "angular": data.cvel[0, 0:3].copy()
                    }
                }
            states.append(body_states)
        return states

    def apply_action(self, u):
        for model, data in self.actor_handles:
            data.ctrl[:] = u

def load_model(urdf_path):
    """Load URDF model"""
    full_path = f"{ASSET_ROOT}/{urdf_path}"
    print(f"Loading asset from '{full_path}'")
    try:
        model = mujoco.MjModel.from_xml_path(full_path)
        return model
    except Exception as e:
        print(f"Error loading URDF: {e}")
        raise

def acquire_sim(robot, dt=0.001):
    """
    Create and configure mujoco simulation
    
    Args:
        dt: control timestep (seconds)
    
    Returns:
        MujocoSimulator: configured simulator object
    """
    sim = MujocoSimulator(dt=dt)
    
    # Create a default mujoco model (will be populated with robot + terrain)
    # We'll create this as empty initially and add content later
    sim.model = load_model(urdf_path=robot)
    
    # Set dt
    sim.model.opt.timestep = dt
    
    # Create initial data
    sim.data = mujoco.MjData(sim.model)
    
    return sim

#MuJoCo 不支援 IsaacGym 那種 env 複製方式
def create_envs(sim, robot, num_envs, envs_per_row, env_spacing):
    """
    Create multiple mujoco environments with robot instances
    
    Args:
        sim: MujocoSimulator object
        robot: RobotType enum
        num_envs: number of environments
        envs_per_row: grid layout (for visualization)
        env_spacing: spacing between environments
    
    Returns:
        envs: list of mujoco models (one per env)
        actor_handles: list of (model, data) tuples for each robot
    """
    
    envs = []
    actor_handles = []
    
    # Load base robot model
    base_model = load_model(robot)
    
    # Create multiple instances
    for i in range(num_envs):
        # For mujoco, we can create separate model instances or use a single model
        # For simplicity, we'll load a new model for each env
        model = load_model(robot)
        data = mujoco.MjData(model)
        
        # Set initial position
        # Find the robot body and set its initial position
        try:
            # Usually the root body is at index 0 after geoms
            # This is a simplified approach - you may need to adjust based on your URDF
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
            if body_id >= 0:
                data.qpos[0] = (i % envs_per_row) * env_spacing  # x position
                data.qpos[1] = (i // envs_per_row) * env_spacing  # y position
                data.qpos[2] = init_height  # z position
                data.qpos[3:7] = [1, 0, 0, 0]  # quaternion (identity)
        except:
            # If we can't find body, use default positions in qpos
            data.qpos[2] = init_height
        
        envs.append(model)
        actor_handles.append((model, data))
    
    return envs, actor_handles

def add_viewer(sim, env, actor):
    """
    Create mujoco viewer
    
    Args:
        sim: MujocoSimulator object
        cam_pos: camera position [x, y, z]
        cam_target: camera target [x, y, z]
    
    Returns:
        viewer: mujoco viewer object
    """
    if not (env or actor):
        print("Warning: No env/actor handles provided for viewer")
        return None
    
    # Use the first environment's model for visualization
    model, data = env, actor
    
    cam_pos = np.array([2.0, 2.0, 2.0])
    cam_target = np.array([0.0, 0.0, 0.0])
    
    viewer = mujoco.viewer.launch_passive(model, data)
    
    # Set camera
    viewer.cam.lookat = cam_target
    viewer.cam.distance = np.linalg.norm(cam_pos)
    viewer.cam.azimuth = np.arctan2(cam_pos[1], cam_pos[0]) * 180 / np.pi
    viewer.cam.elevation = np.arcsin(cam_pos[2] / np.linalg.norm(cam_pos)) * 180 / np.pi
    
    return viewer

def get_dof_count(model):
    """Get number of DOFs"""
    return model.nq


def get_dof_names(model):
    """Get names of all DOFs"""
    dof_names = []
    for i in range(model.nq):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_DOF, i)
        dof_names.append(name if name else f"dof_{i}")
    return dof_names


def set_joint_effort(model, data, effort):
    """
    Set joint effort/torque
    
    Args:
        model: mujoco model
        data: mujoco data
        joint_name: name of joint
        effort: effort value
    """
    joint_name = get_dof_names(model)
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id >= 0:
        # Find the actuator for this joint
        for i in range(model.nu):
            if model.actuator_trnid[i, 0] == joint_id:
                data.ctrl[i] = effort
                break


def set_all_joint_efforts(model, data, efforts):
    data.ctrl[:] = efforts


def get_joint_state(model, data, joint_name):
    """
    Get joint position and velocity
    
    Args:
        model: mujoco model
        data: mujoco data
        joint_name: name of joint
    
    Returns:
        (position, velocity): joint state
    """
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id >= 0:
        qpos_idx = model.jnt_qposadr[joint_id]
        qvel_idx = model.jnt_dofadr[joint_id]
        return data.qpos[qpos_idx], data.qvel[qvel_idx]
    return 0.0, 0.0


def get_body_state(model, data, body_name):
    """
    Get body position and orientation
    
    Args:
        model: mujoco model
        data: mujoco data
        body_name: name of body
    
    Returns:
        pos: position [x, y, z]
        quat: quaternion [w, x, y, z]
        lin_vel: linear velocity
        ang_vel: angular velocity
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id >= 0:
        pos = data.xpos[body_id].copy()
        quat = data.xquat[body_id].copy()
        lin_vel = data.cvel[body_id, 3:6].copy()
        ang_vel = data.cvel[body_id, 0:3].copy()
        return pos, quat, lin_vel, ang_vel
    return None, None, None, None