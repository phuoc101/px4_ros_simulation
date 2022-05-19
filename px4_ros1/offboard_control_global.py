#!/usr/bin/python3
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geoid_height import geoid_height
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
import rospy
import numpy as np
import numpy.linalg as la


class OffboardDroneNode:
    def __init__(self):
        rospy.init_node("offboard_node", anonymous=True, log_level=rospy.INFO)
        rospy.loginfo("Starting OffboardDroneNode.")

        self.ARMED = False
        self.LAND = False

        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self.state_callback)
        self.global_pos_sub = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.global_pos_callback)
        # altitude is measured as above mean sea level (AMSL)
        self.pos_pub = rospy.Publisher(
            "mavros/setpoint_position/global", GeoPoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy(
            "mavros/cmd/arming", CommandBool)
        self.landing_client = rospy.ServiceProxy(
            "mavros/cmd/land", CommandTOL)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # the setpoint publishing rate MUST be faster than 2Hz
        self.rate = rospy.Rate(20.0)

        # control goals
        self.tolerance = 0.1

        # init some variables for subscribed topics and control
        self.k_g2g = 2
        self.initial_global_position = np.zeros(3)
        self.current_global_position = np.zeros(3)
        self.current_state = State()
        self.is_initialized_global_pos = False
        self.step_cnt = -1
        self.mission = np.array([[0, 0, 2], [0, 0, 1], [0, 0, 3]])
        self.mission_steps = self.mission.shape[0]

    def wait_for_FCU_connection(self):
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

    # altitude is measured as above WGS-84 ellipsoid
    def global_pos_callback(self, msg):
        self.current_global_position = np.array(
            [msg.latitude, msg.longitude, msg.altitude])
        if not self.is_initialized_global_pos:
            self.initial_global_position = self.current_global_position
            self.is_initialized_global_pos = True

    def state_callback(self, msg):
        self.current_state = msg
        self.ARMED = msg.armed

    def arm_vehicle(self):
        if self.current_state.mode != "OFFBOARD":
            rospy.wait_for_service('mavros/set_mode')
            rospy.logdebug("Trying to enable offboard...")
            set_mode_response = self.set_mode_client(custom_mode="OFFBOARD")
            if set_mode_response.mode_sent:
                rospy.logdebug("Offboard enabled")
        elif not self.current_state.armed:
            rospy.wait_for_service("mavros/cmd/arming")
            arming_response = self.arming_client(value=True)
            if arming_response.success:
                rospy.logdebug("Vehicle armed")

    def publish_setpoint(self, lat, long, alt):
        pose = GeoPoseStamped()
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.latitude = lat
        pose.pose.position.longitude = long
        pose.pose.position.altitude = alt
        self.pos_pub.publish(pose)

    def land_vehicle(self):
        rospy.wait_for_service("mavros/cmd/land")
        landing_response = self.landing_client()
        if landing_response.success:
            rospy.logdebug("Published command landing")

    def is_at_goal(self, g2g_vect):
        return la.norm(g2g_vect) <= self.tolerance

    def calc_path(self, k=1):
        if self.step_cnt == -1:
            self.step_cnt += 1
            rospy.loginfo(
                f"next goal: {self.initial_global_position + self.mission[self.step_cnt]}")
        if self.step_cnt < self.mission_steps:
            goal = self.initial_global_position + \
                self.mission[self.step_cnt, :]
            g2g_vector = goal - self.current_global_position
            if self.is_at_goal(g2g_vector):
                self.step_cnt += 1
                if self.step_cnt < self.mission_steps:
                    rospy.loginfo(
                        f"next goal: {self.initial_global_position + self.mission[self.step_cnt]}")
        else:
            next_pos = None
            return next_pos
        next_pos = self.current_global_position + k*g2g_vector
        # need to convert between ellipsoid and amsl level, because sensor measures ellipsoid
        # height but publish altitude in amsl (above mean sea level)
        ellipsoid_amsl_conv = geoid_height(
            self.current_global_position[0], self.current_global_position[1])
        next_pos[2] -= ellipsoid_amsl_conv
        return next_pos

    def fly(self):
        self.wait_for_FCU_connection()
        while not rospy.is_shutdown():
            if not self.LAND:
                self.arm_vehicle()
                next_pos = self.calc_path(k=self.k_g2g)
            if next_pos is not None:
                self.publish_setpoint(next_pos[0], next_pos[1], next_pos[2])
            elif not self.LAND:
                self.LAND = True
                rospy.loginfo("landing_vehicle...")
            elif self.current_global_position[2] >= self.initial_global_position[2] + self.tolerance:
                self.land_vehicle()
            else:
                rospy.loginfo("landed, bye")
                break
            self.rate.sleep()


if __name__ == "__main__":
    offboard_node = OffboardDroneNode()
    offboard_node.fly()
