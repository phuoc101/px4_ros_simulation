# source ros packages
sros1() {
  source /opt/ros/noetic/setup.zsh
  # source ~/ros1ws/devel/setup.zsh
  # source ~/img_proj_ws/devel/setup.zsh
  alias tf='cd /var/tmp && rosrun tf view_frames && zathura frames.pdf &'
  # export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/phuoc10/PX4_dev/PX4-Autopilot
  # export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/phuoc10/PX4_dev/PX4-Autopilot/Tools/sitl_gazebo
}

#source ros 2
sros2() {
  source /opt/ros/foxy/setup.zsh
  # source ~/ros2ws/install/setup.zsh
  #export ROS_DOMAIN_ID=25
  # argcomplete for ros2 & colcon
  eval "$(register-python-argcomplete3 ros2)"
  eval "$(register-python-argcomplete3 colcon)"
}

#source px4 ros2
spx4_ros1() {
  sros1
  source ~/px4_dev/PX4-Autopilot/Tools/setup_gazebo.bash ~/px4_dev/PX4-Autopilot ~/px4_dev/PX4-Autopilot/build/px4_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/phuoc10/px4_dev/PX4-Autopilot
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/phuoc10/px4_dev/PX4-Autopilot/Tools/sitl_gazebo
}

#source px4 ros2
spx4_ros2() {
  sros2
  source ~/px4_dev/px4_ros_com_ros2/install/setup.zsh
}

#source moveit workspace
init_moveit() {
  #source /media/phuoc10/KILANBAYBAY/Projects/robot_proj_2021/moveit_proj/ws_moveit/devel/setup.zsh;
  source ~/ws_moveit/devel/setup.zsh;
}

# source turtlebot simulation ros1
init_turtlebot3_ros1() {
  source /media/phuoc10/KILANBAYBAY/Projects/robot_proj_2021/turtlebot3_rospy/catkin_ws/devel/setup.zsh;
  export TURTLEBOT3_MODEL=waffle_pi;
  export MY_MAP_PATH=/media/phuoc10/KILANBAYBAY/Projects/robot_proj_2021/turtlebot3_rospy/slam_map
}

# source turtlebot simulation ros2
init_turtlebot3_ros2() {
  source /media/phuoc10/KILANBAYBAY/Projects/robot_proj_2022/turtlebot3_ws/install/setup.zsh
  source /media/phuoc10/KILANBAYBAY/Projects/robot_proj_2022/jetbot_ws/install/setup.zsh
  export TURTLEBOT3_MODEL=waffle_pi
}

source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/phuoc10/.gazebo/models
