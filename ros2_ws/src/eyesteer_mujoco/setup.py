from setuptools import setup

package_name = "eyesteer_mujoco"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eyesteer",
    maintainer_email="minu.adefris@gmail.com",
    description="MuJoCo hardware backend for the eyesteer ROS 2 stack (mujoco / mujoco_hil)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_bridge = eyesteer_mujoco.mujoco_bridge:main"
        ],
    },
)
