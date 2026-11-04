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
    description='Walker autonomy - drive forward with Nav2, detect obstacles ahead, slow down when the human is not aware',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'obstacle_detector = my_robot_autonomy.obstacle_detector:main',
            'walker_brain = my_robot_autonomy.walker_brain:main',
            'forward_goal = my_robot_autonomy.forward_goal:main',
            'world_markers = my_robot_autonomy.world_markers:main',
        ],
    },
)
