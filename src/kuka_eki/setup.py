import os
from glob import glob

from setuptools import setup

package_name = 'kuka_eki'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juyeol Jeong',
    maintainer_email='MAINTAINER_EMAIL',
    description='EKI communication layer between ROS 2 and a KUKA controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_eki_joint_server = kuka_eki.kuka_eki_joint_server:main',
            'drill_control_server = kuka_eki.drill_control_server:main',
        ],
    },
)
