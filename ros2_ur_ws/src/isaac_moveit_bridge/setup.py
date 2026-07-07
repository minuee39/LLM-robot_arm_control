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
        ],
    },
)
