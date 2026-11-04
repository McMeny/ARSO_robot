from setuptools import setup

package_name = 'my_robot_mujoco'

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
    description='Bridge between the MuJoCo simulation and ROS 2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mujoco_bridge = my_robot_mujoco.mujoco_bridge:main',
        ],
    },
)
