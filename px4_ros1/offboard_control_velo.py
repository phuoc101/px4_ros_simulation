#!/usr/bin/python3
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import rospy
import numpy as np


class OffboardDroneNode:
    def __init__(self):
        rospy.init_node("offboard_node", anonymous=True)
        rospy.loginfo("Starting OffboardDroneNode.")
        self.current_state = State()
        self.current_odom = Odometry()

        self.ARMED = False
        self.LAND = False

        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self.state_callback)
        self.odom_sub = rospy.Subscriber(
            "/mavros/odometry/in", Odometry, self.odom_callback)
        self.pos_pub = rospy.Publisher(
            "mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy(
            "mavros/cmd/arming", CommandBool)
        self.landing_client = rospy.ServiceProxy(
            "mavros/cmd/land", CommandTOL)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # the setpoint publishing rate MUST be faster than 2Hz
        self.rate = rospy.Rate(20.0)

    def wait_for_FCU_connection(self):
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

    def odom_callback(self, msg):
        self.current_odom = msg.pose.pose

    def state_callback(self, msg):
        self.current_state = msg
        self.ARMED = msg.armed

    def arm_vehicle(self):
        if self.current_state.mode != "OFFBOARD":
            rospy.loginfo("Trying to enable offboard...")
            rospy.wait_for_service('mavros/set_mode')
            set_mode_response = self.set_mode_client(custom_mode="OFFBOARD")
            if set_mode_response.mode_sent:
                rospy.loginfo("Offboard enabled")
        elif not self.current_state.armed:
            rospy.wait_for_service("mavros/cmd/arming")
            arming_response = self.arming_client(value=True)
            if arming_response.success:
                rospy.loginfo("Vehicle armed")

    def go_with_velo(self, x, y, z):
        twist = TwistStamped()
        twist.twist.linear.x = x
        twist.twist.linear.y = y
        twist.twist.linear.z = z
        self.pos_pub.publish(twist)

    def landing(self):
        rospy.wait_for_service("mavros/cmd/land")
        landing_response = self.landing_client()
        if landing_response.success:
            rospy.loginfo("Published command landing")

    def fly(self):
        self.wait_for_FCU_connection()
        while not rospy.is_shutdown():
            if not self.LAND:
                self.arm_vehicle()
            if np.abs(self.current_odom.position.z - 2) >= 0.01 and not self.LAND:
                self.go_with_velo(0, 0, 0.7)
            elif not self.LAND:
                self.LAND = True
            elif np.abs(self.current_odom.position.z) >= 0.1:
                self.landing()
            else:
                rospy.loginfo("landed, bye")
                break
            self.rate.sleep()


if __name__ == "__main__":
    offboard_node = OffboardDroneNode()
    offboard_node.fly()
