from setuptools import setup, find_packages
from glob import glob

package_name = 'respeaker_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    url='https://github.com/hello-robot',
    license='Apache License 2.0',
    author='Hello Robot Inc.',
    author_email='support@hello-robot.com',
    description='The respeaker_ros2 package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'respeaker_node = respeaker_ros2.respeaker_node:main',
            'speech_to_text = respeaker_ros2.speech_to_text:main',
        ],
    },
)
