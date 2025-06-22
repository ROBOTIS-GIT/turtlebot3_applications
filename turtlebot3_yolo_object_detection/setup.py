from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_yolo_object_detection'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='1.3.3',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    author='YeonSoo Noh',
    author_email='nys8277@gmail.com',
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'YOLO Object Detection for TurtleBot3 Examples.'
    ),
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_yolo_object_detection =  \
                turtlebot3_yolo_object_detection.turtlebot3_yolo_object_detection:main'
        ],
    },
)
