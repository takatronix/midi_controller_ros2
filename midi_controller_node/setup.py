from setuptools import find_packages, setup

package_name = 'midi_controller_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/midi_controller_launch.py']),
    ],
    install_requires=['setuptools', 'mido', 'python-rtmidi'],
    zip_safe=True,
    maintainer='takatronix',
    maintainer_email='takatronix@gmail.com',
    description='ROS2 node for MIDI controller integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'midi_controller = midi_controller_node.midi_controller:main',
        ],
    },
)
