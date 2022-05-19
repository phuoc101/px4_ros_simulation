# ROS2 imports
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
# OpenCV imports
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
# Common lib imports
import numpy as np
import numpy.linalg as LA
import time
import sys


class FlyDrone(Node):
    def __init__(self):
        super().__init__('takeoffAndLand')
        # control parameters
        self.k_landing = 3
        self.k_gtg = 2 
        self.Ts = 0.1  # seconds
        self.flytime = 10
        # status flags for control loop
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
        self.odom = np.array([0., 0., 0., 0.], np.float32)
        self.takeoff_point = np.array([0., 0., -5., 0.], np.float32)
        self.path = np.array(
            [[0., 10., -5., 0.], [1., 2., -3., 0.], self.takeoff_point],
            np.float32)
        # self.path = np.array([self.takeoffPoint], np.float32)
        # self.path = np.array(
        # [self.takeoffPoint])
        self.tolerance = 0.2
        self.landing_tolerance = 0.1
        self.bridge = CvBridge()
        # Aruco markers parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.aruco_sz = 100
        self.aruco_cen = np.array([-1, -1], np.int16)
        self.frame_cen = np.array([320, 160], np.int16)
        self.land_cnt = 0
        self.img_sz = [640, 320]
        self.timeout_cnt = 0

        """
        ROS2 SUBSCRIBERS
        """
        self.timesyncSub = self.create_subscription(
            Timesync, '/fmu/timesync/out', self.timesync_callback, 10)
        self.vehicleStatusSub = self.create_subscription(
            VehicleControlMode, '/fmu/vehicle_control_mode/out',
            self.vehicleStatus_callback, 10)
        self.odomSub = self.create_subscription(
            VehicleOdometry, '/fmu/vehicle_odometry/out',
            self.odom_callback, 10)
        self.camSub = self.create_subscription(
            Image, 'drone_cam_capture', self.cam_callback, 10)
        """
        ROS2 PUBLISHERS
        """
        self.vehicleCmdPub = self.create_publisher(
            VehicleCommand, '/fmu/vehicle_command/in', 10)
        self.offboardCtrlPub = self.create_publisher(
            OffboardControlMode, 'fmu/offboard_control_mode/in', 10)
        # self.setpointPub = self.create_publisher(
        # TrajectorySetpoint, '/safety_check', 10)
        self.setpointPub = self.create_publisher(
            TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        self.timer = self.create_timer(self.Ts, self.timer_callback)

    def generatePath(self):
        x = np.arange(int(self.flytime/self.Ts))
        y = x**2
        return y

    def cam_callback(self, msg):
        """
        Receive image messages and process them to read Aruco markers
        """
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
            if np.abs(self.odom[2]) < 0.1:
                print("exiting...")
                time.sleep(1)
                sys.exit()

    def landing(self):
        """
        Landing mode
        """
        # publish offboard control mode constantly to keep drone in offboard
        # mode
        self.publishOffboardControlMode()
        nextPoint = self.odom.copy()
        self.publishSetpoint(nextPoint)
        if self.aruco_cen[0] != -1:
            eastOffset = 1/self.img_sz[0]*(self.aruco_cen[0]-self.frame_cen[0])
            northOffset = 1/self.img_sz[1]*(self.frame_cen[1]-self.aruco_cen[1])
            offset = np.array([northOffset, eastOffset], np.float32)
            # offset = offset/LA.norm(offset)
            self.timeout_cnt = 0
            if np.abs(self.odom[2]) > 1:
                print(f"odom current {self.odom}")
                # nextPoint[1] += self.kLanding*eastOffset
                # nextPoint[0] += self.kLanding*northOffset
                nextPoint[0:2] += self.k_landing * offset
                print(
                    f"offset {offset}")
                print(f"frame cen {self.frame_cen}")
                print(f"aruco cen {self.aruco_cen}")
                if LA.norm(offset) >= self.landing_tolerance:
                    self.publishSetpoint(nextPoint)
                else:
                    nextPoint[2] += 0.2
                    print("Moving down")
                    self.publishSetpoint(nextPoint)
            else:
                self.publishVehicleCmd(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
                print('published command LAND, byeeeee')
                self.END = True
        else:
            print("No Aruco found, waiting")
            self.timeout_cnt += 1
            if self.timeout_cnt == 100:
                print("timeout, landing")
                self.publishVehicleCmd(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
                print('published command LAND, byeeeee')
                self.END = True
            else:
                self.publishSetpoint(nextPoint)

    def control(self):
        self.publishOffboardControlMode()
        if not self.ARMED:
            # switch to offboard control
            self.publishVehicleCmd(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # arm drone
            self.publishVehicleCmd(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            print("ARMED cmd sent")
        elif not self.START and self.ARMED and not self.FOLLOW:
            self.START = True
            print("Taking off")
        elif self.START and LA.norm(self.takeoff_point - self.odom) > \
                self.tolerance:
            # takeoff
            # self.publishSetpoint(self.takeoffPoint)
            self.publishSetpoint(self.goToGoal(
                goalSetpoint=self.takeoff_point, k=self.k_gtg))
        elif not self.FOLLOW and LA.norm(self.takeoff_point - self.odom) < \
                self.tolerance:
            self.START = False
            self.FOLLOW = True
            print("following path")
            print(f"next setpoint: {self.path[self.path_cnt]}")
        elif self.path_cnt < len(self.path) and self.FOLLOW:
            # self.publishSetpoint(self.path[self.pathCnt])
            if LA.norm(self.odom - self.path[self.path_cnt]) < self.tolerance:
                self.path_cnt += 1
                if self.path_cnt < len(self.path):
                    print(f"next setpoint: {self.path[self.path_cnt]}")
            else:
                self.publishSetpoint(self.goToGoal(
                    goalSetpoint=self.path[self.path_cnt], k=self.k_gtg))
        elif self.path_cnt == len(self.path) and not self.LAND:
            self.LAND = True
            print('landing')
        # elif self.LAND and not self.END:
            # self.publishVehicleCmd(
            # VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            # print('published command LAND')
            # self.LAND = False
            # self.END = True

    def publishVehicleCmd(self, command, param1, param2=0.0, param3=0.0):
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
        self.vehicleCmdPub.publish(msg)

    def publishOffboardControlMode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboardCtrlPub.publish(msg)

    def publishSetpoint(self, setpoint):
        # unused values are set to NaN by default
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        # NED coordinates => z axis pointing downwards
        msg.x = float(setpoint[0])
        msg.y = float(setpoint[1])
        msg.z = float(setpoint[2])
        msg.yaw = float(setpoint[3])
        self.setpointPub.publish(msg)

    def goToGoal(self, goalSetpoint, k=1):
        currPos = self.odom[0:3].copy()
        goal = goalSetpoint[0:3].copy()
        goToGoalVector = goal - currPos
        nextPos = currPos + k * \
            goToGoalVector / LA.norm(goToGoalVector)
        nextYaw = k * (goalSetpoint[3]-self.odom[3])
        nextSetpoint = np.hstack([nextPos, nextYaw])
        return nextSetpoint

    def vehicleStatus_callback(self, msg):
        if msg.flag_armed:
            self.ARMED = True
        else:
            self.ARMED = False

    def odom_callback(self, msg):
        eulerAngle = quat2euler(msg.q)
        self.odom = np.array([msg.x, msg.y, msg.z, eulerAngle[2]])


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
    main()
