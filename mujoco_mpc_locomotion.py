import sys
import time
import os
import numpy as np
from MPC_Controller.Parameters import Parameters
from MPC_Controller.robot_runner.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.utils import DTYPE
from mujoco_sim import udp_reader
import mujoco
import mujoco.viewer
from mujoco_sim.mujoco_sim_utils import *
from mujoco_sim.sdk2_debug_logger import CsvLogger, add_vec, vec_fields, wall_time_ns
from argparse import ArgumentParser
from MPC_Controller.utils import GaitType, FSM_StateName

parser = ArgumentParser(prog="RL_MPC_LOCOMOTION")
parser.add_argument("--disable-gamepad", action="store_true")
parser.add_argument("--render-fps", type=int, default=60, help="render fps")
parser.add_argument("--debug-log", action="store_true", help="write MuJoCo/MPC debug CSV logs")
parser.add_argument("--debug-log-dir", default="logs/mujoco_mpc_debug", help="directory for debug CSV logs")
args = parser.parse_args()
use_gamepad = not args.disable_gamepad
if use_gamepad:
    gamepad = udp_reader.UDPGamepad(port=9876)


def init_mpc_debug_logger(log_dir):
    os.makedirs(log_dir, exist_ok=True)
    fields = (
        ["wall_time_ns", "sim_time", "cmd_vx", "cmd_vy", "cmd_wz"]
        + vec_fields("q", 12)
        + vec_fields("dq", 12)
        + vec_fields("tau_cmd", 12)
        + vec_fields("ctrl", 12)
        + vec_fields("imu_quat", 4)
        + vec_fields("imu_gyro", 3)
        + vec_fields("body_pos", 3)
        + vec_fields("body_vel", 3)
        + vec_fields("se_rpy", 3)
        + vec_fields("se_pos", 3)
        + vec_fields("se_vbody", 3)
        + vec_fields("se_omega_body", 3)
        + vec_fields("mpc_x", 12)
        + vec_fields("mpc_x_des", 12)
        + vec_fields("mpc_u_grf", 12)
        + vec_fields("foot_p", 12)
        + vec_fields("foot_p_des", 12)
        + vec_fields("foot_v_des", 12)
        + vec_fields("contact_state", 4)
        + vec_fields("swing_state", 4)
        + vec_fields("mpc_table", 40)
        + ["foot_normal_FR", "foot_normal_FL", "foot_normal_RR", "foot_normal_RL"]
        + ["foot_force_norm_FR", "foot_force_norm_FL", "foot_force_norm_RR", "foot_force_norm_RL"]
    )
    print(f"[MuJoCo MPC Debug] writing logs to {log_dir}")
    return CsvLogger(os.path.join(log_dir, "controller_mpc.csv"), fields)


def get_foot_contact_forces(model, data):
    foot_geom_ids = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in ("FR", "FL", "RR", "RL")
    }
    normal = {name: 0.0 for name in foot_geom_ids}
    norm = {name: 0.0 for name in foot_geom_ids}
    force = np.zeros(6, dtype=np.float64)
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        foot_name = None
        for name, geom_id in foot_geom_ids.items():
            if contact.geom1 == geom_id or contact.geom2 == geom_id:
                foot_name = name
                break
        if foot_name is None:
            continue
        mujoco.mj_contactForce(model, data, contact_id, force)
        normal[foot_name] += abs(float(force[0]))
        norm[foot_name] += float(np.linalg.norm(force[:3]))
    return normal, norm


def log_mpc_debug(logger, model, data, robot_runner, commands, leg_torques):
    if logger is None:
        return
    se = robot_runner._stateEstimator.getResult()
    cMPC = robot_runner._controlFSM.statesList.locomotion.cMPC
    snap = getattr(cMPC, "debug_snapshot", {})
    foot_normal, foot_norm = get_foot_contact_forces(model, data)
    row = {
        "wall_time_ns": wall_time_ns(),
        "sim_time": float(data.time),
        "cmd_vx": float(commands[0]),
        "cmd_vy": float(commands[1]),
        "cmd_wz": float(commands[2]),
        "foot_normal_FR": foot_normal["FR"],
        "foot_normal_FL": foot_normal["FL"],
        "foot_normal_RR": foot_normal["RR"],
        "foot_normal_RL": foot_normal["RL"],
        "foot_force_norm_FR": foot_norm["FR"],
        "foot_force_norm_FL": foot_norm["FL"],
        "foot_force_norm_RR": foot_norm["RR"],
        "foot_force_norm_RL": foot_norm["RL"],
    }
    add_vec(row, "q", data.sensordata[0:12], 12)
    add_vec(row, "dq", data.sensordata[12:24], 12)
    add_vec(row, "tau_cmd", leg_torques, 12)
    add_vec(row, "ctrl", data.ctrl[:12], 12)
    add_vec(row, "imu_quat", data.sensordata[36:40], 4)
    add_vec(row, "imu_gyro", data.sensordata[40:43], 3)
    add_vec(row, "body_pos", data.sensordata[46:49], 3)
    add_vec(row, "body_vel", data.sensordata[49:52], 3)
    add_vec(row, "se_rpy", se.rpy.flatten(), 3)
    add_vec(row, "se_pos", se.position.flatten(), 3)
    add_vec(row, "se_vbody", se.vBody.flatten(), 3)
    add_vec(row, "se_omega_body", se.omegaBody.flatten(), 3)
    for key, count in (
        ("mpc_x", 12),
        ("mpc_x_des", 12),
        ("mpc_u_grf", 12),
        ("foot_p", 12),
        ("foot_p_des", 12),
        ("foot_v_des", 12),
        ("contact_state", 4),
        ("swing_state", 4),
        ("mpc_table", 40),
    ):
        add_vec(row, key, snap.get(key, []), count)
    logger.write(row)


def main():
    global target
    dt = Parameters.controller_dt
    model = mujoco.MjModel.from_xml_path("assets/go2/scene.xml")
    model.opt.timestep = dt
    data = mujoco.MjData(model)

    # Launch MuJoCo passive viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.cam.lookat = [0.0, 0.0, 0.0]
    viewer.cam.distance = 2.0
    viewer.cam.azimuth  = 90
    viewer.cam.elevation = -20

    # Set up MPC controller
    robotRunner = RobotRunnerFSM()
    robotRunner.init(RobotType.GO2)

    count = 0
    render_fps = args.render_fps
    render_count = max(1, int(1 / render_fps / dt))

    running_time = 0.0
    legTorques = np.zeros(12, dtype=DTYPE)
    log_file = init_csv_logger("mujoco_log.csv")
    debug_logger = init_mpc_debug_logger(args.debug_log_dir) if args.debug_log else None

    while viewer.is_running():
        step_start = time.time()
        running_time += dt
        commands = np.zeros(3, dtype=DTYPE)

        if use_gamepad:
            if gamepad.is_standing:
                legTorques = pd_stand(data, running_time)
                data.ctrl[:] = legTorques

            if gamepad.is_moving:
                lin_speed, ang_speed, e_stop = gamepad.get_command()
                Parameters.cmpc_gait = gamepad.get_gait()
                Parameters.control_mode = gamepad.get_mode()
                if not e_stop:
                    commands = np.array([lin_speed[0], lin_speed[1], ang_speed], dtype=DTYPE)

                # run controllers
                dof_states = get_dof_state(data)
                body_states = get_body_state(data)
                legTorques = robotRunner.run(dof_states, body_states, commands).astype(np.float32)
                log_mpc_debug(debug_logger, model, data, robotRunner, commands, legTorques)

                fl_foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "FL_foot")
                fl_foot_z = data.xpos[fl_foot_id, 2]
                log_mpc_states(log_file, data.time, robotRunner, fl_foot_z, legTorques)
                data.ctrl[:] = legTorques[LEG_MJC_TO_MPC]

        if Parameters.locomotionUnsafe:
            gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
            Parameters.locomotionUnsafe = False

        mujoco.mj_step(model, data)

        if count % render_count == 0:
            if model.nq >= 3:
                viewer.cam.lookat[0] = data.qpos[0]
                viewer.cam.lookat[1] = data.qpos[1]
                viewer.cam.lookat[2] = data.qpos[2]
            viewer.sync()

        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

        count += 1

    if log_file is not None:
        log_file.flush()
        log_file.close()
        print("\n[Logger] CSV 檔案已成功儲存！")
    if debug_logger is not None:
        debug_logger.close()

    if use_gamepad:
        gamepad.stop()

    sys.exit(0)

if __name__=="__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
        sys.exit(0)
