from setuptools import setup

package_name = 'my_robot_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='mcmeny',
    maintainer_email='mcmeny@todo.todo',
    description='Smart walker - hazard loop, tentacles, head-guided direction, floor trajectory, SSVEP decoder',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tentacles = my_robot_autonomy.tentacles:main',
            'hazard_tracker = my_robot_autonomy.hazard_tracker:main',
            'walker_brain = my_robot_autonomy.walker_brain:main',
            'tentacle_driver = my_robot_autonomy.tentacle_driver:main',
            'trajectory_overlay = my_robot_autonomy.trajectory_overlay:main',
            'eeg_decoder = my_robot_autonomy.eeg_decoder:main',
            'muse_emulator = my_robot_autonomy.muse_emulator:main',
            'eeg_check = my_robot_autonomy.eeg_check:main',
            'session_logger = my_robot_autonomy.session_logger:main',
            'world_markers = my_robot_autonomy.world_markers:main',
            'grip_state_node = my_robot_autonomy.grip_state_node:main',
            'tentacle_planner_node = my_robot_autonomy.tentacle_planner_node:main',
            'ssvep_emulator = my_robot_autonomy.ssvep_emulator:main',
            'walker_keys = my_robot_autonomy.walker_keys:main',
            'run_scenario = my_robot_autonomy.run_scenario:main',
        ],
    },
)
