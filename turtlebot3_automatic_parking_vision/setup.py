from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_automatic_parking_vision'
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
    zip_safe=True,
    author=['Gilbert', 'ChanHyeong Lee'],
    author_email=['kkjong@robotis.com', 'dddoggi1207@gmail.com'],
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
        'Automatic Parking Vision for TurtleBot3 Examples.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_automatic_parking_vision = \
                turtlebot3_automatic_parking_vision.turtlebot3_automatic_parking_vision:main',
        ],
    },
)
