from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np

from controller.pick_place import PickPlaceController
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from tasks.pick_place import PickPlace
from llm_to_json import parse_user_command_with_llm, select_llm_provider


from command_parser import (
    parse_user_command,
    validate_command,
    command_to_target_position,
)

from scene_config import (
    BLOCK_SIZE,
    OBJECT_POSITIONS,
    OBJECT_COLORS,
)
from vision.mock_vision import MockVision
from vision.scene_objects import detections_to_scene_objects

# =========================
# 1. 기본 설정
# =========================

TASK_NAME = "ur10e_pick_place"

USE_GEMINI_API = True
USE_CHATGPT_API = False

try:
    llm_provider = select_llm_provider(USE_GEMINI_API, USE_CHATGPT_API)
except ValueError as error:
    print("[설정 오류]", error)
    simulation_app.close()
    raise SystemExit from error

vision = MockVision()


def get_scene_objects_from_vision() -> dict:
    detections = vision.detect_objects()
    return detections_to_scene_objects(detections)


def update_object_positions_from_sim(world, object_names) -> dict:
    positions = {}
    for object_name in object_names:
        scene_object = world.scene.get_object(object_name)
        position, _ = scene_object.get_world_pose()
        positions[object_name] = position
    return positions


scene_objects = get_scene_objects_from_vision()

# =========================
# 2. 사용자 명령 입력 및 검증
# =========================

max_retry = 3
command = None
target_position = None
picking_position = None


def parse_command(user_text: str) -> dict:
    if llm_provider is None:
        print("[Parser] 로컬 명령 파서 사용")
        return parse_user_command(user_text)

    try:
        command = parse_user_command_with_llm(
            user_text,
            scene_objects,
            provider=llm_provider,
        )
        print(f"[Parser] {llm_provider} LLM 사용")
        return command

    except Exception as error:
        print(f"[LLM 경고] {llm_provider}를 사용할 수 없어 로컬 명령 파서로 전환합니다.")
        print("[LLM 오류 내용]", error)
        print("[Parser] 로컬 명령 파서 사용")
        return parse_user_command(user_text)


def wait_for_valid_command() -> dict:
    global scene_objects, target_position

    for attempt in range(max_retry):
        try:
            user_text = input("로봇 명령을 입력하세요: ")

            if user_text.strip().lower() in ["종료", "exit", "quit"]:
                simulation_app.close()
                raise SystemExit

            new_command = parse_command(user_text)
            current_scene_objects = get_scene_objects_from_vision()
            validate_command(new_command, current_scene_objects)

            scene_objects = current_scene_objects
            target_position = command_to_target_position(command=new_command, scene_objects=scene_objects)

            print("Parsed command:", new_command)
            return new_command

        except ValueError as e:
            print("[명령 오류]", e)

            remaining = max_retry - attempt - 1
            if remaining > 0:
                print(f"다시 입력해주세요. 남은 시도 횟수: {remaining}")
            else:
                raise

    raise ValueError("명령 입력 실패")


command = wait_for_valid_command()

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
)



# 중요: task를 반드시 World에 등록해야 함
my_world.add_task(my_task)


# =========================
# 4. 기준 object 추가
# =========================
# 현재는 red_block / blue_block을 기준 물체로 사용
# 나중에 실제로 집을 물체로 쓰려면 picking_position도 command 기반으로 바꿔야 함

for object_name, object_pos in OBJECT_POSITIONS.items():
    my_world.scene.add(
        DynamicCuboid(
            prim_path=f"/World/{object_name}",
            name=object_name,
            position=object_pos,
            scale=BLOCK_SIZE,
            color=OBJECT_COLORS[object_name],
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
    gripper=my_ur10e.gripper
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

        pick_object = my_world.scene.get_object(command["pick_object"])

        pick_position, _ = pick_object.get_world_pose()
        placing_position = target_position

        actions = my_controller.forward(
            picking_position=pick_position,
            placing_position=placing_position,
            current_joint_positions=my_ur10e.get_joint_positions(),
            end_effector_offset=np.array([0, 0, 0.20]),
        )

        if my_controller.is_done():
            if not done_printed:
                print("done picking and placing")
                done_printed = True

                try:
                    runtime_object_positions = update_object_positions_from_sim(
                        my_world,
                        OBJECT_POSITIONS.keys(),
                    )
                    vision.update_object_positions(runtime_object_positions)
                    scene_objects = get_scene_objects_from_vision()

                    command = wait_for_valid_command()
                    my_controller.reset()
                    done_printed = False
                except ValueError:
                    print("새 명령을 받지 못해 종료합니다.")
                    simulation_app.close()
                    raise SystemExit

            continue

        articulation_controller.apply_action(actions)

    if my_world.is_stopped():
        reset_needed = True

simulation_app.close()
