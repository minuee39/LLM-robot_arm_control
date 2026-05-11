from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np

from controller.pick_place import PickPlaceController
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from tasks.pick_place import PickPlace

from command_parser import (
    parse_user_command,
    validate_command,
    command_to_target_position,
)


# =========================
# 1. 기본 설정
# =========================

TASK_NAME = "ur10e_pick_place"

blue_block_pos = np.array([0.3, 0.6, 0.02575])
red_block_pos = np.array([-0.3, 0.6, 0.02575])
cube_initial_position = np.array([0.5, 0.0, 0.02575])

scene_objects = {
    "cube": {
        "position": cube_initial_position
    },
    "target": {
        "position": np.array([0.1, 0.8, 0.02575])
    },
    "blue_block": {
        "position": blue_block_pos
    },
    "red_block": {
        "position": red_block_pos
    },
}


# =========================
# 2. 사용자 명령 입력 및 검증
# =========================

max_retry = 3
command = None
target_position = None

for attempt in range(max_retry):
    try:
        user_text = input("로봇 명령을 입력하세요: ")

        command = parse_user_command(user_text)
        validate_command(command, scene_objects)
        target_position = command_to_target_position(command, scene_objects)

        print("Parsed command:", command)
        print("Target position:", target_position)
        break

    except ValueError as e:
        print("[명령 오류]", e)

        remaining = max_retry - attempt - 1
        if remaining > 0:
            print(f"다시 입력해주세요. 남은 시도 횟수: {remaining}")
        else:
            print("명령을 이해하지 못해 로봇을 실행하지 않습니다.")
            simulation_app.close()
            raise SystemExit


# =========================
# 3. World 및 PickPlace Task 생성
# =========================

my_world = World(
    stage_units_in_meters=1.0,
    physics_dt=1 / 200,
    rendering_dt=20 / 200,
)



my_task = PickPlace(
    name=TASK_NAME,
    cube_initial_position=cube_initial_position,
    target_position=target_position,
    cube_size=np.array([0.1, 0.0515, 0.1]),
)

# 중요: task를 반드시 World에 등록해야 함
my_world.add_task(my_task)


# =========================
# 4. 기준 object 추가
# =========================
# 현재는 red_block / blue_block을 기준 물체로 사용
# 나중에 실제로 집을 물체로 쓰려면 picking_position도 command 기반으로 바꿔야 함

my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/red_block",
        name="red_block",
        position=red_block_pos,
        scale=np.array([0.1, 0.0515, 0.1]),
        color=np.array([1.0, 0.0, 0.0]),
    )
)

my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/blue_block",
        name="blue_block",
        position=blue_block_pos,
        scale=np.array([0.1, 0.0515, 0.1]),
        color=np.array([0.0, 0.0, 1.0]),
    )
)


# =========================
# 5. World 초기화 및 로봇/컨트롤러 가져오기
# =========================

my_world.reset()

task_params = my_world.get_task(TASK_NAME).get_params()

ur10e_name = task_params["robot_name"]["value"]
my_ur10e = my_world.scene.get_object(ur10e_name)

my_controller = PickPlaceController(
    name="controller",
    robot_articulation=my_ur10e,
    gripper=my_ur10e.gripper,
)

articulation_controller = my_ur10e.get_articulation_controller()

reset_needed = False
done_printed = False


# =========================
# 6. Simulation Loop
# =========================

while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
            my_controller.reset()
            done_printed = False

        if my_world.current_time_step_index == 0:
            my_controller.reset()
            done_printed = False

        observations = my_world.get_observations()

        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=target_position,
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0, 0, 0.20]),
        )

        if my_controller.is_done():
            if not done_printed:
                print("done picking and placing")
                done_printed = True
            continue

        articulation_controller.apply_action(actions)

    if my_world.is_stopped():
        reset_needed = True


simulation_app.close()