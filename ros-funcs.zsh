# source ros noetic packages
sros1() {
  source /opt/ros/noetic/setup.zsh
  alias tf='cd /var/tmp && rosrun tf view_frames && zathura frames.pdf &'
}

#source ros 2 foxy packages
sros2() {
  source /opt/ros/foxy/setup.zsh
  eval "$(register-python-argcomplete3 ros2)"
  eval "$(register-python-argcomplete3 colcon)"
}

#source px4 ros1
spx4_ros1() {
  sros1
  source $HOME/px4_dev/PX4-Autopilot/Tools/setup_gazebo.bash ~/px4_dev/PX4-Autopilot ~/px4_dev/PX4-Autopilot/build/px4_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/px4_dev/PX4-Autopilot
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/px4_dev/PX4-Autopilot/Tools/sitl_gazebo
}

#source px4 ros2
spx4_ros2() {
  sros2
  source $HOME/px4_dev/px4_ros_com_ros2/install/setup.zsh
}

source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/.gazebo/models
