import os
import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from mujoco_sim.unitree_sdk2py_bridge import UnitreeSdk2Bridge
from mujoco_sim import config_sdk2 as config
from mujoco_sim.sdk2_debug_logger import CsvLogger, wall_time_ns

_DEBUG_LOG     = os.environ.get("SDK2_DEBUG_LOG", "0") == "1"
_DEBUG_LOG_DIR = os.environ.get("SDK2_DEBUG_LOG_DIR", "logs/sdk2_debug")

locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path("assets/go2/scene.xml")
mj_data = mujoco.MjData(mj_model)

viewer = mujoco.viewer.launch_passive(mj_model, mj_data)
viewer.cam.lookat[:] = [0.0, 0.0, 0.25]
viewer.cam.distance = 2.0
viewer.cam.azimuth = 90
viewer.cam.elevation = -20

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    # ── debug logger ──────────────────────────────────────────────────
    bridge_logger = None
    last_lowcmd_ns = [wall_time_ns()]   # list so the closure can mutate it

    if _DEBUG_LOG:
        os.makedirs(_DEBUG_LOG_DIR, exist_ok=True)
        bridge_logger = CsvLogger(
            os.path.join(_DEBUG_LOG_DIR, "bridge_sim.csv"),
            ["wall_time_ns", "sim_step", "lowcmd_age_ms"],
        )
        def _track_lowcmd(msg: LowCmd_):
            last_lowcmd_ns[0] = wall_time_ns()
        lowcmd_tracker = ChannelSubscriber("rt/lowcmd", LowCmd_)
        lowcmd_tracker.Init(_track_lowcmd, 10)
        print(f"[Bridge Debug] writing bridge_sim.csv to {_DEBUG_LOG_DIR}")
    # ──────────────────────────────────────────────────────────────────
    sim_step = 0

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()
        mujoco.mj_step(mj_model, mj_data)
        locker.release()

        if bridge_logger is not None:
            now = wall_time_ns()
            bridge_logger.write({
                "wall_time_ns":  now,
                "sim_step":      sim_step,
                "lowcmd_age_ms": (now - last_lowcmd_ns[0]) / 1e6,
            })
        sim_step += 1

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    if bridge_logger is not None:
        bridge_logger.close()


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.cam.lookat[0] = mj_data.qpos[0]
        viewer.cam.lookat[1] = mj_data.qpos[1]
        #viewer.cam.lookat[2] = mj_data.qpos[2]
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
