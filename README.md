# youbot_project
Youbot MoveIt! project, to launch


Add to catkin_ws and compile packages:

[youbot_description](https://github.com/mikhail-chirkov/youbot_description)

[youbot_moveit](https://github.com/mikhail-chirkov/youbot_moveit)

[youbot_navigation](https://github.com/mikhail-chirkov/youbot_navigation)

[youbot_simulation](https://github.com/mikhail-chirkov/youbot_simulation)

Also [Grasp fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin) and [Gazebo link attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher) need to  be installed.

To launch:
> roslaunch youbot_project youbot.launch load_grasp_fix:=true 
