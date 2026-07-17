from llm_to_json import parse_user_command_with_llm, select_llm_provider

from command_server import RobotCommandServer
from command_parser import (
    MEMORY_OBJECT_ALIASES,
    parse_user_command_with_memory,
    validate_command,
)
from grasp_policy import FixedGraspPolicy, GraspRequest
from scene_manager import SceneManager

from scene_config import (
    BLOCK_SIZE,
    OBJECT_POSITIONS,
    OBJECT_COLORS,
)
from vision.mock_vision import MockVision

simulation_app = None

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
    if simulation_app is not None:
        simulation_app.close()
    raise SystemExit from error

vision = MockVision()
scene_manager = SceneManager.from_defaults()
grasp_policy = FixedGraspPolicy()


def update_object_positions_from_sim(world, object_names) -> dict:
    positions = {}
    for object_name in object_names:
        scene_object = world.scene.get_object(object_name)
        position, _ = scene_object.get_world_pose()
        positions[object_name] = position
    return positions


scene_manager.update_from_detections(vision.detect_objects())
scene_objects = scene_manager.as_command_scene()

# =========================
# 2. 사용자 명령 입력 및 검증
# =========================

command = None
target_position = None


def parse_command(user_text: str) -> dict:
    if any(alias in user_text for alias in MEMORY_OBJECT_ALIASES):
        print("[Parser] memory-aware 로컬 명령 파서 사용")
        return parse_user_command_with_memory(user_text, scene_manager.memory)

    if llm_provider is None:
        print("[Parser] 로컬 명령 파서 사용")
        return parse_user_command_with_memory(user_text, scene_manager.memory)

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
        return parse_user_command_with_memory(user_text, scene_manager.memory)


def build_valid_command(user_text: str) -> dict:
    global scene_objects, target_position

    new_command = parse_command(user_text)
    scene_manager.update_from_detections(vision.detect_objects())
    current_scene_objects = scene_manager.as_command_scene()
    validate_command(new_command, current_scene_objects)

    scene_objects = current_scene_objects
    target_position = scene_manager.resolve_target_position(new_command)

    print("Parsed command:", new_command)
    return new_command

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from controller.pick_place import PickPlaceController
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from tasks.pick_place import PickPlace

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

command_server = RobotCommandServer()
command_server.start()
command_host, command_port = command_server.address

print("[READY] Isaac scene loaded. Waiting for commands from another terminal.")
print(f"[READY] Command server: {command_host}:{command_port}")
print("[READY] Example:")
print(f'  python3 scripts/send_robot_command.py "빨간 블럭을 파란 블럭 옆에 둬"')
print("  python3 scripts/send_robot_command.py")

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

        if command is None:
            incoming_command = command_server.get_next_command()
            if incoming_command is None:
                continue

            if incoming_command.strip().lower() in {"종료", "exit", "quit"}:
                print("[INFO] 종료 명령을 받았습니다.")
                break

            try:
                command = build_valid_command(incoming_command)
                my_controller.reset()
                done_printed = False
            except ValueError as error:
                print("[명령 오류]", error)
                command = None
                target_position = None
                continue

        pick_object = my_world.scene.get_object(command["pick_object"])

        pick_position, _ = pick_object.get_world_pose()
        scene_manager.update_object(command["pick_object"], pick_position)
        placing_position = target_position
        grasp_action = grasp_policy.predict(
            GraspRequest(
                object_name=command["pick_object"],
                object_pose=pick_position,
                target_position=placing_position,
                object_size=scene_manager.get_object(command["pick_object"]).size,
                object_type=command["pick_object"],
                robot_state={"joint_positions": my_ur10e.get_joint_positions()},
            )
        )

        actions = my_controller.forward(
            picking_position=pick_position,
            placing_position=placing_position,
            current_joint_positions=my_ur10e.get_joint_positions(),
            end_effector_offset=grasp_action.end_effector_offset,
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
                    scene_manager.update_from_detections(vision.detect_objects())
                    scene_manager.apply_pick_place_result(command, target_position)
                    scene_objects = scene_manager.as_command_scene()

                    command = None
                    target_position = None
                    my_controller.reset()
                    done_printed = False
                except ValueError:
                    print("작업 완료 후 scene 상태를 갱신하지 못해 종료합니다.")
                    simulation_app.close()
                    raise SystemExit

            continue

        articulation_controller.apply_action(actions)

    if my_world.is_stopped():
        reset_needed = True

command_server.shutdown()
simulation_app.close()
