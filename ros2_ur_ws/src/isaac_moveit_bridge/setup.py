from setuptools import find_packages, setup


package_name = "isaac_moveit_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minwoo",
    maintainer_email="minwoo@example.com",
    description="MoveIt action bridge for Isaac Sim",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "action_bridge = isaac_moveit_bridge.action_bridge:main",
            "joint_state_logger = isaac_moveit_bridge.joint_state_logger:main",
            "moveit_pick_place_demo = isaac_moveit_bridge.moveit_pick_place_demo:main",
            "moveit_pose_goal = isaac_moveit_bridge.moveit_pose_goal:main",
            "moveit_target_follow = isaac_moveit_bridge.moveit_target_follow:main",
            "tcp_pose_test = isaac_moveit_bridge.tcp_pose_test:main",
        ],
    },
)
