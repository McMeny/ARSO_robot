import os
from glob import glob

from setuptools import setup

package_name = 'walker_vr'


def web_files():
    """Install everything under web/ to share/walker_vr/web/, keeping the folders."""
    out = []
    for folder, _, files in os.walk('web'):
        if files:
            out.append((os.path.join('share', package_name, folder), [os.path.join(folder, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + web_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcmeny',
    maintainer_email='mcmeny@todo.todo',
    description='The VR side of the smart walker - headset app, spectator and operator pages, HTTPS/WebSocket bridge',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'web_server = walker_vr.web_server:main',
            'make_certs = walker_vr.make_certs:main',
        ],
    },
)
