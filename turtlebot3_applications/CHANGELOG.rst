^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3_applications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2025-06-23)
------------------
* Fix package.xml in turtlebot3_panorama
* Contributors: Hyungyu Kim

1.3.2 (2025-06-20)
------------------
* Add pytest in setup.py
* Contributors: Hyungyu Kim

1.3.1 (2025-06-20)
------------------
* Fix cmake error in turtlebot3_panorama
* Contributors: Hyungyu Kim

1.3.0 (2025-06-04)
------------------
* Added turtlebot3_yolo_object_detection pkg
* Added a Python script that runs the YOLO model on live camera images and publishes detection results with bounding boxes
* Contributors: YeonSoo Noh

1.2.0 (2025-04-25)
------------------
* Support for ROS 2 Humble version
* Added turtlebot3_aruco_tracker pkg
* Update turtlebot3_automatic_parking_vision pkg to use turtlebot3_aruco_tracker
* Developed follower pkg by applying a new algorithm using Nav2
* Uses odom topic to estimate yaw rotation and compute angles between snapshots
* Contributors: ChanHyeong Lee, Hyungyu Kim, YeonSoo Noh

1.1.0 (2019-01-23)
------------------
* added launch file for automatic_parking node `#22 <https://github.com/ROBOTIS-GIT/turtlebot3_applications/issues/22>`_
* created launch and removed .py
* modified follower
* added follower node runnable from anywhere `#25 <https://github.com/ROBOTIS-GIT/turtlebot3_applications/issues/25>`_
* Contributors: Daniel Ingram, Gilbert, Darby Lim, Pyo

1.0.0 (2018-05-29)
------------------
* added turtlebot3_automatic_parking
* added turtlebot3_automatic_parking_vision
* deleted unused msg
* modified msg to use turtlebot3_applications_msg package
* merged pull request `#18 <https://github.com/ROBOTIS-GIT/turtlebot3_applications/issues/18>`_ `#17 <https://github.com/ROBOTIS-GIT/turtlebot3_applications/issues/17>`_ `#16 <https://github.com/ROBOTIS-GIT/turtlebot3_applications/issues/16>`_ `#14 <https://github.com/ROBOTIS-GIT/turtlebot3_applications/issues/14>`_
* Contributors: Darby Lim, Leon Jung, Gilbert, Pyo

0.2.0 (2018-04-20)
------------------
* added turtlebot3 automatic parking vision example source code (turtlebot3_automatic_parking_vision)
* changes to ar_marker_alvar from ar_pose package (turtlebot3_automatic_parking_vision)
* fixed recovering method of automatic parking using vision (turtlebot3_automatic_parking_vision)
* added angles of center, start, end into spot filter (turtlebot3_automatic_parking)
* Contributors: Leon Jung, Gilbert, Pyo

0.1.0 (2018-03-14)
------------------
* added turtlebot3_automatic_parking pkg
* added turtlebot3_follow_filter pkg
* added turtlebot3_follower pkg
* added turtlebot3_automatic_parking pkg
* refactoring to release
* Contributors: Ashe, Chris, Leon Jung, Darby Lim, Gilbert, Pyo
