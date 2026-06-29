from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
import numpy as np
from controller.pick_place import PickPlaceController
from isaacsim.core.api import World
from tasks.pick_place import PickPlace

my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=20 / 200)

from command_parser import parse_user_command, command_to_target_position

user_text = input("로봇 명령을 입력하세요: ")

command = parse_user_command(user_text)
target_position = command_to_target_position(command)

print("Parsed command:", command)
print("Target position:", target_position)

my_task = PickPlace(name="ur10e_pick_place", target_position=target_position, cube_size=np.array([0.1, 0.0515, 0.1]))
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("ur10e_pick_place").get_params()
ur10e_name = task_params["robot_name"]["value"]
my_ur10e = my_world.scene.get_object(ur10e_name)
# initialize the controller

my_controller = PickPlaceController(name="controller", robot_articulation=my_ur10e, gripper=my_ur10e.gripper)
task_params = my_world.get_task("ur10e_pick_place").get_params()
articulation_controller = my_ur10e.get_articulation_controller()

reset_needed = False

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
            my_controller.reset()
        if my_world.current_time_step_index == 0:
            my_controller.reset()

        observations = my_world.get_observations()
        # forward the observation values to the controller to get the actions
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            # This offset needs tuning as well
            end_effector_offset=np.array([0, 0, 0.20]),
        )
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)

    if my_world.is_stopped():
        reset_needed = True


simulation_app.close()