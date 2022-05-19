#!/usr/bin/python3
from geometry_msgs.msg import PoseStamped
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
            "/mavros/global_position/local", Odometry, self.odom_callback)
        self.pos_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy(
            "mavros/cmd/arming", CommandBool)
        self.landing_client = rospy.ServiceProxy(
            "mavros/cmd/land", CommandTOL)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # the setpoint publishing rate MUST be faster than 2Hz
        self.rate = rospy.Rate(20.0)

        # control goals
        self.tolerance = 0.05

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

    def publish_setpoint(self, x, y, z):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.pos_pub.publish(pose)

    def landing(self):
        rospy.wait_for_service("mavros/cmd/land")
        landing_response = self.landing_client()
        if landing_response.success:
            rospy.loginfo("Published command landing")

    def fly(self):
        self.wait_for_FCU_connection()
        while not rospy.is_shutdown():
            if not self.ARMED:
                self.arm_vehicle()
            if np.abs(self.current_odom.position.z - 2) >= self.tolerance and not self.LAND:
                self.publish_setpoint(0, 0, 2)
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
