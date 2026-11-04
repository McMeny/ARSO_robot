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
            'forward_goal = my_robot_autonomy.forward_goal:main',
            'trajectory_overlay = my_robot_autonomy.trajectory_overlay:main',
            'eeg_decoder = my_robot_autonomy.eeg_decoder:main',
            'muse_emulator = my_robot_autonomy.muse_emulator:main',
            'eeg_check = my_robot_autonomy.eeg_check:main',
            'session_logger = my_robot_autonomy.session_logger:main',
            'world_markers = my_robot_autonomy.world_markers:main',
        ],
    },
)
