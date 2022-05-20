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
        rospy.init_node("offboard_node", log_level=rospy.INFO)
        rospy.loginfo("Starting OffboardDroneNode.")

        self.ARMED = False
        self.LAND = False

        self.__state_sub = rospy.Subscriber(
            "/mavros/state", State, self.__state_callback)
        self.__global_pos_sub = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.__global_pos_callback)
        # altitude is measured as above mean sea level (AMSL)
        self.__pos_pub = rospy.Publisher(
            "mavros/setpoint_position/global", GeoPoseStamped, queue_size=10)
        self.__arming_client = rospy.ServiceProxy(
            "mavros/cmd/arming", CommandBool)
        self.__landing_client = rospy.ServiceProxy(
            "mavros/cmd/land", CommandTOL)
        self.__set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # the setpoint publishing rate MUST be faster than 2Hz
        self._rate = rospy.Rate(20.0)

        # control goals
        self._tolerance = 0.1

        # init some variables for subscribed topics and control
        self._k_g2g = 0.7
        self.__initial_global_position = np.zeros(3)
        self.__current_global_position = np.zeros(3)
        self.__current_state = State()
        self.__is_initialized_global_pos = False
        self.__step_cnt = -1
        self.__mission_offset = np.array([[0, 0, 2], [0, 0, 1], [0, 0, 3]])
        self._mission = np.zeros(self.__mission_offset.shape)
        self.__mission_steps = self._mission.shape[0]

    def __wait_for_FCU_connection(self):
        while not rospy.is_shutdown() and not self.__current_state.connected:
            self._rate.sleep()

    # altitude is measured as above WGS-84 ellipsoid
    def __global_pos_callback(self, msg):
        self.__current_global_position = np.array(
            [msg.latitude, msg.longitude, msg.altitude])
        if not self.__is_initialized_global_pos:
            self.__initial_global_position = self.__current_global_position
            self._mission = self.__initial_global_position + self.__mission_offset
            self.__is_initialized_global_pos = True

    def __state_callback(self, msg):
        self.__current_state = msg
        self.ARMED = msg.armed

    def __arm_vehicle(self):
        if self.__current_state.mode != "OFFBOARD":
            rospy.wait_for_service('mavros/set_mode')
            rospy.logdebug("Trying to enable offboard...")
            set_mode_response = self.__set_mode_client(custom_mode="OFFBOARD")
            if set_mode_response.mode_sent:
                rospy.logdebug("Offboard enabled")
        elif not self.__current_state.armed:
            rospy.wait_for_service("mavros/cmd/arming")
            arming_response = self.__arming_client(value=True)
            if arming_response.success:
                rospy.logdebug("Vehicle armed")

    def __publish_setpoint(self, lat, long, alt):
        pose = GeoPoseStamped()
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.latitude = lat
        pose.pose.position.longitude = long
        pose.pose.position.altitude = alt
        self.__pos_pub.publish(pose)

    def land_vehicle(self):
        rospy.wait_for_service("mavros/cmd/land")
        landing_response = self.__landing_client()
        if landing_response.success:
            rospy.logdebug("Published command landing")

    def __is_at_goal(self, g2g_vect):
        return la.norm(g2g_vect) <= self._tolerance

    def __calc_path(self, k=1):
        current_global_position = self.__current_global_position.copy()
        if self.__step_cnt == -1:
            self.__step_cnt += 1
            rospy.loginfo(f"Start mission")
            rospy.loginfo(f"next goal: {self._mission[self.__step_cnt]}")
        if self.__step_cnt < self.__mission_steps:
            g2g_vector = self._mission[self.__step_cnt, :] - \
                current_global_position
            if self.__is_at_goal(g2g_vector):
                self.__step_cnt += 1
                if self.__step_cnt < self.__mission_steps:
                    rospy.loginfo(
                        f"next goal: {self._mission[self.__step_cnt]}")
        else:
            next_pos = None
            return next_pos
        next_pos = current_global_position + k*g2g_vector/la.norm(g2g_vector)
        # need to convert between ellipsoid and amsl level, because sensor measures ellipsoid
        # height but publish altitude in amsl (above mean sea level)
        ellipsoid_amsl_conv = geoid_height(
            current_global_position[0], current_global_position[1])
        next_pos[2] -= ellipsoid_amsl_conv
        return next_pos

    def fly(self):
        self.__wait_for_FCU_connection()
        while not rospy.is_shutdown():
            if not self.LAND:
                self.__arm_vehicle()
                next_pos = self.__calc_path(k=self._k_g2g)
            if next_pos is not None:
                self.__publish_setpoint(next_pos[0], next_pos[1], next_pos[2])
            elif not self.LAND:
                self.LAND = True
                self.land_vehicle()
                rospy.loginfo("landing_vehicle...")
            elif self.__current_global_position[2] >= self.__initial_global_position[2] + self._tolerance:
                rospy.logdebug("vehicle is moving down")
            else:
                rospy.loginfo("landed, bye")
                break
            self._rate.sleep()


if __name__ == "__main__":
    offboard_node = OffboardDroneNode()
    offboard_node.fly()
