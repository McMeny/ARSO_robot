from setuptools import setup

package_name = "eyesteer_behavior"

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
    description="Mode manager, wander, admittance, anchor bridge, safety, health monitor, MCU bridge, scenario player",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mode_manager = eyesteer_behavior.mode_manager_node:main",
            "localization_monitor = eyesteer_behavior.localization_monitor_node:main",
            "wander = eyesteer_behavior.wander_node:main",
            "admittance = eyesteer_behavior.admittance_node:main",
            "anchor_bridge = eyesteer_behavior.anchor_bridge_node:main",
            "safety = eyesteer_behavior.safety_node:main",
            "health_monitor = eyesteer_behavior.health_monitor_node:main",
            "mcu_bridge = eyesteer_behavior.mcu_bridge_node:main",
            "scan_mask = eyesteer_behavior.scan_mask_node:main",
            "scenario_player = eyesteer_behavior.scenario_player:main",
            "web_server = eyesteer_behavior.web_server:main"
        ],
    },
)
