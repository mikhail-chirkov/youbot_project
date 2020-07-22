# youbot_project
Youbot MoveIt! project, based on gazebo simulation of youbot and generated moveit package. To fix some problems with gazebo physics [Grasp fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin) and [Gazebo link attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher) are used. Tested ONLY for ROS Kinetic.

Video with demonstration: https://youtu.be/Sb7gXdOLEjI

To install add to catkin_ws and compile packages:

[youbot_description](https://github.com/mikhail-chirkov/youbot_description)

[youbot_moveit](https://github.com/mikhail-chirkov/youbot_moveit)

[youbot_navigation](https://github.com/mikhail-chirkov/youbot_navigation)

[youbot_simulation](https://github.com/mikhail-chirkov/youbot_simulation)

Also [Grasp fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin) and [Gazebo link attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher) need to  be installed.

To launch use:

`roslaunch youbot_project youbot.launch load_grasp_fix:=true`
