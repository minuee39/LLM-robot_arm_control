from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.storage.native import get_assets_root_path


class PickPlace(tasks.BaseTask):
    def __init__(
        self,
        name: str = "ur10e_pick_place",
        offset: Optional[np.ndarray] = None,
    ) -> None:
        super().__init__(name=name, offset=offset)
        self._robot = None

    def set_up_scene(self, scene) -> None:
        super().set_up_scene(scene)

        scene.add_default_ground_plane()

        self._robot = self.set_robot()
        scene.add(self._robot)

    def set_robot(self) -> SingleManipulator:
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise Exception("Could not find Isaac Sim assets folder")

        asset_path = (
            assets_root_path
            + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd"
        )

        add_reference_to_stage(
            usd_path=asset_path,
            prim_path="/ur",
        )

        gripper = ParallelGripper(
            end_effector_prim_path="/ur/ee_link/robotiq_arg2f_base_link",
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([0]),
            joint_closed_positions=np.array([40]),
            action_deltas=np.array([-40]),
            use_mimic_joints=True,
        )

        manipulator = SingleManipulator(
            prim_path="/ur",
            name="ur10_robot",
            end_effector_prim_path="/ur/ee_link/robotiq_arg2f_base_link",
            gripper=gripper,
        )

        return manipulator

    def get_observations(self) -> dict:
        joint_positions = self._robot.get_joint_positions()

        return {
            self._robot.name: {
                "joint_positions": joint_positions,
            }
        }

    def get_params(self) -> dict:
        return {
            "robot_name": {
                "value": self._robot.name
            }
        }