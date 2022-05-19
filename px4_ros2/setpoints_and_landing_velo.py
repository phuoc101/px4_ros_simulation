import rclpy
from rclpy.node import Node
from transforms3d.euler import quat2euler
from sensor_msgs.msg import Image
from px4_msgs.msg import Timesync
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleControlMode

import cv2
from cv2 import aruco
from cv_bridge import CvBridge

import numpy as np
import numpy.linalg as LA
import time
import sys
import matplotlib.pyplot as plt
import logging


class FlyDrone(Node):
    def __init__(self):
        super().__init__('takeoff_and_land')
        ### Control parameters ###
        self.k_p = 1
        self.k_i = 0.05
        self.k_d = 1
        self.k_p_landing = 3
        self.k_i_landing = 0.1
        self.k_d_landing = 0.05
        self.t_s = 0.1  # seconds
        self.prev_err = 0
        self.pid_p = np.zeros(3)
        self.pid_i = np.zeros(3)
        self.pid_d = np.zeros(3)
        self.hover = 0
        self.u = np.zeros(3)
        ### Plotting variables ###
        self.start_time = time.time()
        self.time_his_nav = [0]
        self.odom_his_nav = [np.zeros(3)]
        self.goal_his_nav = [np.zeros(3)]
        self.time_his_land = []
        self.err_his_land = []
        ### status flags for control loop ###
        # set to true by vehicle vehicle_status_callback if drone actually armed
        self.ARMED = False
        # signal when drone took off
        self.START = False
        # signal when drone following path
        self.FOLLOW = False
        self.path_cnt = 0
        # signal when drone landed
        self.LAND = False
        self.END = False
        self.timestamp = 0
        # self.odom = np.array([0., 0., 0., 0.], np.float32)
        # self.takeoffPoint = np.array([0., 0., -5., 0.], np.float32)
        self.odom = np.array([0., 0., 0.], np.float32)
        self.takeoff_point = np.array([0., 0., -5.], np.float32)
        self.path = np.array(
            [self.takeoff_point, [0., 10., -5.],
                [1., 2., -3.], self.takeoff_point],
            np.float32)
        # self.path = np.array([self.takeoffPoint], np.float32)
        # self.path = np.array([self.takeoffPoint])
        self.tolerance = 0.5
        self.landing_tolerance = 0.02
        self.bridge = CvBridge()
        ### Aruco markers parameters ###
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.aruco_sz = 100
        self.aruco_cen = np.array([-1, -1], np.int16)
        self.frame_cen = np.array([320, 160], np.int16)
        self.land_cnt = 0
        self.img_sz = [640, 320]
        self.timeout_cnt = 0

        ### ROS2 SUBSCRIBERS ###
        self.timesync_sub = self.create_subscription(
            Timesync, '/fmu/timesync/out', self.timesync_callback, 10)
        self.vehicle_status_sub = self.create_subscription(
            VehicleControlMode, '/fmu/vehicle_control_mode/out',
            self.vehicle_status_callback, 10)
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/vehicle_odometry/out',
            self.odomCallback, 10)
        self.cam_sub = self.create_subscription(
            Image, 'drone_cam_capture', self.cam_callback, 10)
        ### ROS2 PUBLISHERS ###
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/vehicle_command/in', 10)
        self.offboard_ctrl_pub = self.create_publisher(
            OffboardControlMode, 'fmu/offboard_control_mode/in', 10)
        # self.setpointPub = self.create_publisher(
        # TrajectorySetpoint, '/safety_check', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        self.timer = self.create_timer(self.t_s, self.timer_callback)

    def cam_callback(self, msg):
        ### Receive image messages and process them to read Aruco markers ###
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected, = aruco.detectMarkers(gray, self.aruco_dict)
        if ids is not None:
            # draw marks on corners
            aruco.drawDetectedMarkers(frame, corners)
            # rvec_all, tvec_all, _obj_points = \
            # aruco.estimatePoseSingleMarkers(corners, self.aruco_sz)
            for idx, marker_id in enumerate(ids):
                # aruco.drawAxis(frame, np.eye(3), np.eye(3),\
                # rvec_all[idx], tvec_all[idx], 100)
                cv2.putText(frame, f'{marker_id}',
                            (int(corners[idx][0][0][0]-30),
                             int(corners[idx][0][0][1])),
                            cv2.FONT_HERSHEY_PLAIN, 2,
                            (255, 0, 0), 2, cv2.LINE_AA)
            self.aruco_cen[0] = np.mean(corners[0][0, :, 0])
            self.aruco_cen[1] = np.mean(corners[0][0, :, 1])
            frame = cv2.circle(frame, (self.aruco_cen[0], self.aruco_cen[1]),
                               radius=3, color=(0, 0, 255), thickness=-1)
        else:
            self.aruco_cen = np.array([-1, -1])
        frame = cv2.circle(frame, (self.frame_cen[0], self.frame_cen[1]),
                           radius=3, color=(0, 0, 255), thickness=-1)
        cv2.imshow('drone cam', frame)
        cv2.waitKey(1)

    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

    def timer_callback(self):
        if not self.LAND:
            self.control()
        elif not self.END:
            self.landing()
        else:
            if np.abs(self.odom[2]) < 0.3:
                logging.info("exiting...")
                self.plot_his()
                sys.exit()

    def landing(self):
        ### Landing mode ###
        # publish offboard control mode constantly to keep drone in offboard
        # mode
        self.publishOffboardControlMode()
        nextPoint = self.odom.copy()
        # self.publishSetpoint(nextPoint)
        if self.aruco_cen[0] != -1:
            eastOffset = 1/self.img_sz[0]*(self.aruco_cen[0]-self.frame_cen[0])
            northOffset = 1/self.img_sz[1] * \
                (self.frame_cen[1]-self.aruco_cen[1])
            offset = np.array([northOffset, eastOffset, 0], np.float32)
            # offset = offset/LA.norm(offset)
            self.timeout_cnt = 0
            if np.abs(self.odom[2]) > 1.2:
                self.err_his_land.append(offset)
                self.time_his_land.append(time.time() - self.start_time)
                logging.debug(f"odom current {self.odom}")
                # nextPoint[1] += self.kLanding*eastOffset
                # nextPoint[0] += self.kLanding*northOffset
                # nextPoint[0:2] += self.kLanding * offset
                nextU = self.pid(offset, self.k_p_landing,
                                 self.k_i_landing, self.k_d_landing)
                logging.debug(f"frame cen {self.frame_cen}")
                logging.debug(f"aruco cen {self.aruco_cen}")
                logging.debug(f"err norm: {LA.norm(offset[0:2])}")
                if LA.norm(offset[0:2]) >= self.landing_tolerance:
                    nextU[2] = self.hover
                    logging.debug(f"u: {nextU}")
                    self.publishSetpoint(nextU)
                else:
                    self.hover += 0.02
                    nextU[2] = self.hover
                    logging.debug(f"u: {nextU}")
                    logging.debug(f"u: {nextU}")
                    logging.debug("Moving down")
                    self.publishSetpoint(nextU)
            else:
                self.publish_vehicle_cmd(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
                logging.info('published command LAND, byeeeee')
                self.END = True
        else:
            logging.warning("No Aruco found, waiting")
            self.timeout_cnt += 1
            if self.timeout_cnt == 100:
                logging.warning("timeout, landing")
                self.publish_vehicle_cmd(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
                logging.info('published command LAND, byeeeee')
                self.END = True
            else:
                self.publishSetpoint(nextPoint)

    def control(self):
        self.publishOffboardControlMode()
        if not self.ARMED:
            # switch to offboard control
            self.publish_vehicle_cmd(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # arm drone
            self.publish_vehicle_cmd(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            logging.info("ARMED cmd sent")
        elif not self.START and self.ARMED and not self.FOLLOW:
            self.START = True
            logging.info("Taking off")
        elif self.START and LA.norm(self.takeoff_point[0:3] - self.odom[0:3]) > \
                self.tolerance:
            # takeoff
            # self.publishSetpoint(self.takeoffPoint)
            self.u = self.goToGoal(
                goalSetpoint=self.takeoff_point, kp=self.k_p,
                ki=self.k_i, kd=self.k_d)
            self.publishSetpoint(self.u)
        elif not self.FOLLOW and LA.norm(self.takeoff_point[0:3] - self.odom[0:3]) < \
                self.tolerance:
            self.START = False
            self.FOLLOW = True
            self.pid_i = 0
            logging.info("following path")
            logging.info(f"next setpoint: {self.path[self.path_cnt]}")
        elif self.path_cnt < len(self.path) and self.FOLLOW:
            if LA.norm(self.odom[0:3] - self.path[self.path_cnt][0:3]) < self.tolerance:
                self.path_cnt += 1
                if self.path_cnt < len(self.path):
                    self.pid_i = 0
                    logging.info(f"next setpoint: {self.path[self.path_cnt]}")
            else:
                self.u = self.goToGoal(
                    goalSetpoint=self.path[self.path_cnt], kp=self.k_p,
                    ki=self.k_i, kd=self.k_d)
                self.publishSetpoint(self.u)
        elif self.path_cnt == len(self.path) and not self.LAND:
            self.pid_i = 0
            self.prev_err = 0
            self.hover = self.odom[2]
            self.LAND = True
            logging.info('landing')
        if not self.LAND:
            self.time_his_nav.append(time.time()-self.start_time)
            self.odom_his_nav.append(self.odom)
            if self.path_cnt < len(self.path):
                self.goal_his_nav.append(self.path[self.path_cnt])
            else:
                self.goal_his_nav.append(self.path[self.path_cnt-1])
        # elif self.LAND and not self.END:
            # self.publishVehicleCmd(
            # VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            # self.LAND = False
            # self.END = True

    def plot_his(self):
        t_his_nav = np.asarray(self.time_his_nav)
        odom_his_nav = np.asarray(self.odom_his_nav)
        g_his_nav = np.asarray(self.goal_his_nav)
        fig_nav, ax_nav = plt.subplots(3)
        ax_nav[0].plot(t_his_nav, odom_his_nav[:, 0], 'r-', label="N")
        ax_nav[0].plot(t_his_nav, g_his_nav[:, 0], 'k--', label="N_goal")
        ax_nav[0].set(xlabel="time (s)", ylabel="N")
        ax_nav[0].legend()
        ax_nav[0].grid()

        ax_nav[1].plot(t_his_nav, odom_his_nav[:, 1], 'b-', label="E")
        ax_nav[1].plot(t_his_nav, g_his_nav[:, 1], 'k--', label="E_goal")
        ax_nav[1].set(xlabel="time (s)", ylabel="E")
        ax_nav[1].legend()
        ax_nav[1].grid()

        ax_nav[2].plot(t_his_nav, odom_his_nav[:, 2], 'g-', label="D")
        ax_nav[2].plot(t_his_nav, g_his_nav[:, 2], 'k--', label="D_goal")
        ax_nav[2].set(xlabel="time (s)", ylabel="D")
        ax_nav[2].legend()
        ax_nav[2].grid()

        t_his_land = np.asarray(self.time_his_land)
        err_his_land = np.asarray(self.err_his_land)
        fig_land, ax_land = plt.subplots(2)
        ax_land[0].plot(t_his_land, err_his_land[:, 0], 'r-', label='err_N')
        ax_land[0].plot(t_his_land, np.zeros(len(err_his_land)), 'k--', label="desired err")
        ax_land[0].set(xlabel="time (s)", ylabel="err_N")
        ax_land[0].legend()
        ax_land[0].grid()

        ax_land[1].plot(t_his_land, err_his_land[:, 1], 'b-', label='err_E')
        ax_land[1].plot(t_his_land, np.zeros(len(err_his_land)), 'k--', label="desired err")
        ax_land[1].set(xlabel="time (s)", ylabel="err_E")
        ax_land[1].legend()
        ax_land[1].grid()

        plt.show()

    def publish_vehicle_cmd(self, command, param1, param2=0.0, param3=0.0):
        # publish specific commands, eg land, takeoff
        msg = VehicleCommand()
        msg.timestamp = self.timestamp

        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    def publishOffboardControlMode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_ctrl_pub.publish(msg)

    def publishSetpoint(self, u):
        # unused values are set to NaN by default
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        # NED coordinates => z axis pointing downwards
        # msg.x = float(setpoint[0])
        # msg.y = float(setpoint[1])
        # msg.z = float(setpoint[2])
        # msg.yaw = float(setpoint[3])
        msg.vx = float(u[0])
        msg.vy = float(u[1])
        msg.vz = float(u[2])
        # msg.yawspeed = float(u[3])
        self.setpoint_pub.publish(msg)

    def pid(self, err, kp, ki, kd):
        self.pid_p = kp * err
        self.pid_i = self.pid_i + ki*err
        self.pid_d = kd * (err-self.prev_err)/self.t_s
        pid_u = self.pid_p + self.pid_i + self.pid_d
        self.prev_err = err
        # pidU[3] = ((pidU[3] + np.pi) % (2*np.pi)) - np.pi
        return pid_u

    def goToGoal(self, goalSetpoint, kp, ki, kd):
        currPos = self.odom.copy()
        goal = goalSetpoint.copy()
        err = goal - currPos
        # err[3] = ((err[3] + np.pi) % (2*np.pi)) - np.pi
        nextU = self.pid(err, kp, ki, kd)
        for i in range(len(nextU)):
            if np.abs(nextU[i]) > 10:
                nextU[i] = np.sign(nextU[i])*10
        return nextU

    def vehicle_status_callback(self, msg):
        if msg.flag_armed:
            self.ARMED = True
        else:
            self.ARMED = False

    def odomCallback(self, msg):
        euler_angle = quat2euler(msg.q)
        euler_angle = np.asarray(euler_angle)
        # eulerAngle[2] = ((eulerAngle[2] + np.pi) % (2*np.pi)) - np.pi
        # self.odom = np.array([msg.x, msg.y, msg.z, eulerAngle[2]])
        self.odom = np.array([msg.x, msg.y, msg.z])


def main(args=None):
    rclpy.init(args=args)
    fly_drone = FlyDrone()
    rclpy.spin(fly_drone)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fly_drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    logging.basicConfig(format='[%(levelname)s]: %(message)s', level=logging.INFO)
    main()
