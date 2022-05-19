## COMMANDS \
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"\
source ~/px4_dev/PX4-Autopilot/Tools/setup_gazebo.bash ~/px4\_dev/PX4-Autopilot ~/px4\_dev/PX4-Autopilot/build/px4\_sitl\_default\
export ROS\_PACKAGE\_PATH=\$ROS\_PACKAGE\_PATH:/home/phuoc10/px4\_dev/PX4-Autopilot|
export ROS\_PACKAGE\_PATH=\$ROS\_PACKAGE\_PATH:/home/phuoc10/px4\_dev/PX4-Autopilot/Tools/sitl_gazebo\
roslaunch px4 mavros_posix_sitl.launch\
python3 mission\_test.py MC\_mission\_box.plan\

## NOTES\
1. For global control, need to set the header message \
header = std\_msgs.msg.Header()\
header.stamp = rospy.Time.now()\
else it won't work\
2. When controlling the FCU using global setpoints, you specify the altitude as meters above mean sea level (AMSL). But when sensing the global position, the altitude reported by ~global\_position/global is specified as meters above the WGS-84 ellipsoid. This can lead to differences in altitude that are dozens of meters apart.\
If you want to go from AMSL to ellipsoid height, add the geoid\_height value.\
To go from ellipsoid height to AMSL, subtract the geoid\_height value.\
