import math
from MPC_Controller.Parameters import Parameters
from MPC_Controller.robot_runner.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.robot_runner.RobotRunnerMin import RobotRunnerMin
from MPC_Controller.robot_runner.RobotRunnerPolicy import RobotRunnerPolicy
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.utils import DTYPE, ControllerType
from RL_Environment import gamepad_reader
import mujoco
from mujoco_sim_utils import *
from argparse import ArgumentParser

parser = ArgumentParser(prog="RL_MPC_LOCOMOTION")

parser.add_argument("--robot", default="Aliengo", choices=[name.title() for name in RobotType.__members__.keys()], help="robot types")
parser.add_argument("--mode", default="Fsm", choices=[name.title() for name in ControllerType.__members__.keys()], help="controller types")
parser.add_argument("--num-envs", type=int, default=1, help="the number of robots")
parser.add_argument("--render-fps", type=int, default=30, help="render fps")
parser.add_argument("--disable-gamepad", action="store_true")
parser.add_argument("--checkpoint", default=None)

args = parser.parse_args()

use_gamepad = not args.disable_gamepad
debug_vis = False # draw ground normal vector

if use_gamepad:
    gamepad = gamepad_reader.Gamepad(vel_scale_x=2.5, vel_scale_y=1.5, vel_scale_rot=3.0)


def main():
    robot = RobotType[args.robot.upper()]
    dt =  Parameters.controller_dt
    sim = acquire_sim(robot, dt) #xml urdf?
    
    # set up the env grid
    num_envs = args.num_envs
    envs_per_row = int(math.sqrt(args.num_envs))
    env_spacing = 0.5
    # one actor per env #create duplicates, method needs a closer look
    envs, actors = create_envs(sim, robot, num_envs, envs_per_row, env_spacing)
    
    viewer = add_viewer(sim, envs[0], actors[0])

    # Setup MPC Controller
    controllers = []
    for _ in range(len(envs)):
        # Setup MPC Controller
        controller_type = ControllerType[args.mode.upper()]
        if controller_type is ControllerType.FSM:
            robotRunner = RobotRunnerFSM()
        elif controller_type is ControllerType.MIN:
            robotRunner = RobotRunnerMin()
        elif controller_type is ControllerType.POLICY:
            robotRunner = RobotRunnerPolicy(args.checkpoint)
        else:
            raise Exception("Invalid ControllerType!")

        robotRunner.init(robot)
        controllers.append(robotRunner)

    # simulation loop
    while viewer.is_running():
        # step the physics
        sim.step()

        # current_time = gym.get_sim_time(sim)
        commands = np.zeros(3, dtype=DTYPE)
        if use_gamepad:
            lin_speed, ang_speed, e_stop = gamepad.get_command()
            Parameters.cmpc_gait = gamepad.get_gait()
            Parameters.control_mode = gamepad.get_mode()
            if not e_stop:
                commands = np.array([lin_speed[0], lin_speed[1], ang_speed], dtype=DTYPE)

        # run controllers
        dof_states = sim.get_state()
        body_states = sim.get_body_state()  
        # run MPC
        torques = controller.run(dof_states, body_states, commands).astype(np.float32)

        # apply
        data.ctrl[:] = torques         

        if Parameters.locomotionUnsafe:
            gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
            Parameters.locomotionUnsafe = False

        if debug_vis:
            pos_np = np.asarray([p for p in body_states["pose"]["p"]], dtype=np.float32)
            gym.add_lines(viewer, envs[0], 1, 
                [pos_np, pos_np + controllers[0]._stateEstimator.result.ground_normal_world], 
                [[255,0,0]])

        if count % render_count == 0:
            # update the viewer
            count = 0
            viewer.sync()

        # Wait for dt to elapse in real time.
        gym.sync_frame_time(sim)
        count += 1

    if use_gamepad:
        gamepad.stop()
        # gamepad.read_thread.join()
        # print("Gamepad read thread killed!") # too slow

    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)

if __name__=="__main__":
    main()