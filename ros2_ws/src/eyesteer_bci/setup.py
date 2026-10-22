from setuptools import setup

package_name = "eyesteer_bci"

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
    description="SSVEP decoding from LSL (Muse + Oz), Muse LSL emulator, calibration service",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ssvep_node = eyesteer_bci.ssvep_node:main",
            "muse_emulator = eyesteer_bci.muse_emulator:main_ros",
            "lsl_recorder = eyesteer_bci.lsl_recorder:main"
        ],
    },
)
