from setuptools import setup

package_name = 'kuka_task_client'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juyeol Jeong',
    maintainer_email='MAINTAINER_EMAIL',
    description=(
        'Command dispatcher and perception for natural-language-driven '
        'drilling: subscribes to /kuka/command, executes deterministic '
        'routines, reports on /kuka/status.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_topic_mover = kuka_task_client.kuka_topic_mover:main',
            'depth_plane_normal = kuka_task_client.depth_plane_normal:main',
        ],
    },
)
