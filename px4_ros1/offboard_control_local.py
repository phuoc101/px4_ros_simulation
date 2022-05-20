#!/usr/bin/python3
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
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
        self.__odom_sub = rospy.Subscriber(
            "/mavros/global_position/local", Odometry, self.__local_pos_callback)
        self.__pos_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.__arming_client = rospy.ServiceProxy(
            "mavros/cmd/arming", CommandBool)
        self.__landing_client = rospy.ServiceProxy(
            "mavros/cmd/land", CommandTOL)
        self.__set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # the setpoint publishing rate MUST be faster than 2Hz
        self._rate = rospy.Rate(20.0)

        # control goals
        self._tolerance = 0.2
        self._k_g2g = 0.7
        self.__step_cnt = -1
        self.mission = np.array([[0, 0, 2], [2, 1, 1], [-1, -2, 3], [0, 0, 5]])
        self.__mission_steps = self.mission.shape[0]
        self.__current_state = State()
        self.__current_local_pos = np.zeros(3)

    def wait_for_FCU_connection(self):
        while not rospy.is_shutdown() and not self.__current_state.connected:
            self._rate.sleep()

    def __local_pos_callback(self, msg):
        current_pos = msg.pose.pose.position
        self.__current_local_pos = np.array(
            [current_pos.x, current_pos.y, current_pos.z])

    def __state_callback(self, msg):
        self.__current_state = msg
        self.ARMED = msg.armed

    def __arm_vehicle(self):
        if self.__current_state.mode != "OFFBOARD":
            rospy.logdebug("Trying to enable offboard...")
            rospy.wait_for_service('mavros/set_mode')
            set_mode_response = self.__set_mode_client(custom_mode="OFFBOARD")
            if set_mode_response.mode_sent:
                rospy.logdebug("Offboard enabled")
        elif not self.__current_state.armed:
            rospy.wait_for_service("mavros/cmd/arming")
            arming_response = self.__arming_client(value=True)
            if arming_response.success:
                rospy.logdebug("Vehicle armed")

    def __publish_setpoint(self, x, y, z):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.__pos_pub.publish(pose)

    def __land_vehicle(self):
        rospy.wait_for_service("mavros/cmd/land")
        landing_response = self.__landing_client()
        if landing_response.success:
            rospy.logdebug("Published command landing")

    def __is_at_goal(self, g2g_vect):
        return la.norm(g2g_vect) <= self._tolerance

    def __calc_path(self, k=1):
        current_local_pos = self.__current_local_pos.copy()
        if self.__step_cnt == -1:
            self.__step_cnt += 1
            rospy.loginfo(
                f"next goal: {self.mission[self.__step_cnt]}")
        if self.__step_cnt < self.__mission_steps:
            goal = self.mission[self.__step_cnt, :]
            g2g_vector = goal - current_local_pos
            if self.__is_at_goal(g2g_vector):
                self.__step_cnt += 1
                if self.__step_cnt < self.__mission_steps:
                    rospy.loginfo(
                        f"next goal: {self.mission[self.__step_cnt]}")
        else:
            next_pos = None
            return next_pos
        next_pos = current_local_pos + k*g2g_vector/la.norm(g2g_vector)
        return next_pos

    def fly(self):
        self.wait_for_FCU_connection()
        next_pos = None
        while not rospy.is_shutdown():
            if not self.LAND:
                self.__arm_vehicle()
                next_pos = self.__calc_path(k=self._k_g2g)
            if next_pos is not None:
                self.__publish_setpoint(next_pos[0], next_pos[1], next_pos[2])
            elif not self.LAND:
                self.LAND = True
                rospy.loginfo("landing vehicle...")
                self.__land_vehicle()
            elif np.abs(self.__current_local_pos[2]) >= self._tolerance:
                rospy.logdebug("vehicle is moving down")
            else:
                rospy.loginfo("landed, bye")
                break
            self._rate.sleep()


if __name__ == "__main__":
    offboard_node = OffboardDroneNode()
    offboard_node.fly()
