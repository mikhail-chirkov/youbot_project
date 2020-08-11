# youbot_project

Demonstration: https://youtu.be/Sb7gXdOLEjI

This is Youbot and MoveIt! project, based on gazebo simulation. Robot is able to perform pick&place movements by using MoveIt!. There also a SLAM functionality impemented.

To fix some problems with gazebo physics [Grasp fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin) and [Gazebo link attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher) are used. Tested ONLY for ROS Kinetic.

To install add to catkin_ws and compile these packages:

[youbot_description](https://github.com/mikhail-chirkov/youbot_description)

[youbot_moveit](https://github.com/mikhail-chirkov/youbot_moveit)

[youbot_navigation](https://github.com/mikhail-chirkov/youbot_navigation)

[youbot_simulation](https://github.com/mikhail-chirkov/youbot_simulation)

Also [Grasp fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin) and [Gazebo link attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher) need to  be installed.
In order to launch this particular simulation scenario [model_editor_models](https://github.com/mikhail-chirkov/model_editor_models) folder should be added in a home(/~) folder and sourced in gasebo as a model database.

To launch the simulation scenario use:

`roslaunch youbot_project youbot.launch load_grasp_fix:=true`

To launch the manipulation script:

`rosrun youbot_project main_script.py`
