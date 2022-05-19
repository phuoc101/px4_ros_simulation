import rclpy
from rclpy.node import Node
import time
import numpy as np

from px4_msgs.msg import Timesync
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
# from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleControlMode


class FlyDrone(Node):
    def __init__(self):
        super().__init__('takeoffAndLand')
        # status flags for control loop
        # set to true by vehicle vehicle_status_callback if drone actually armed
        self.ARMED = False
        # signal when drone took off
        self.START = False
        # signal when drone landed
        self.LAND = False
        self.END = False
        self.timestamp = 0
        self.odom = np.array([0., 0., 0.])
        self.takeoffPoint = np.array([0, 0, -5])

        """
        ROS2 SUBSCRIBERS
        """
        self.timesyncSub = self.create_subscription(
            Timesync, '/fmu/timesync/out', self.timesyncCallback, 10)
        self.vehicleStatusSub = self.create_subscription(
            VehicleControlMode, '/fmu/vehicle_control_mode/out',
            self.vehicleStatusCallback, 10)
        self.odomSub = self.create_subscription(
            VehicleOdometry, '/fmu/vehicle_odometry/out',
            self.odomCallback, 10)
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
        timerPeriod = 0.1  # in seconds
        self.timer = self.create_timer(timerPeriod, self.timerCallback)

    def timesyncCallback(self, msg):
        self.timestamp = msg.timestamp

    def timerCallback(self):
        self.control()

    def control(self):
        self.pubOffboardControlMode()
        if not self.ARMED:
            # switch to offboard control
            self.publishVehicleCmd(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # arm drone
            self.publishVehicleCmd(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            print("ARMED cmd sent")
        if not self.START and self.ARMED:
            # takeoff
            # self.publishVehicleCmd(
            # VehicleCommand.VEHICLE_CMD_NAV_START, 1.0)
            self.START = True
            print("Taking off")
        if self.START and np.linalg.norm(self.takeoffPoint - self.odom) > 1:
            self.pubSetpoint(self.takeoffPoint)
        if not self.LAND and np.linalg.norm(self.takeoffPoint - self.odom) < 1:
            self.LAND = True
            print('landing')
        if self.LAND and not self.END:
            self.publishVehicleCmd(
                VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
        if self.odom[2] < 0.0001 and self.LAND and not self.END:
            self.END = True
            print("landed, done")


        # if self.START and not self.LAND and not self.END:
            # self.publishVehicleCmd(
            # VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            # self.START = False
            # print("Landing")
            # self.LAND = True
        # if self.LAND:
            # self.LAND = False
            # print("Mission completed, chillin'")
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

    def pubOffboardControlMode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboardCtrlPub.publish(msg)

    def pubSetpoint(self, setpoint):
        # unused values should be set to NaN
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        # msg.x = float(x)
        # msg.y = float(y)
        # msg.z = float(-z) #z axis points down in PX4 - NED coordinates
        msg.x = float(setpoint[0])
        msg.y = float(setpoint[1])
        # z axis points down in PX4 - NED coordinates
        msg.z = float(setpoint[2])
        # msg.yaw = float(setpoint[3])  # [-PI:PI] #not set for now
        # msg.vx = 1.0
        # msg.vy = 0.0
        # msg.vz = 1.0
        # msg.acceleration[0] = float("nan")
        # msg.acceleration[1] = float("nan")
        # msg.acceleration[2] = float("nan")
        # msg.jerk[0] = float("nan")
        # msg.jerk[1] = float("nan")
        # msg.jerk[2] = float("nan")
        # msg.thrust[0] = float("nan")
        # msg.thrust[1] = float("nan")
        # msg.thrust[2] = float("nan")
        self.setpointPub.publish(msg)

    def vehicleStatusCallback(self, msg):
        if msg.flag_armed:
            self.ARMED = True

    def odomCallback(self, msg):
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
    main()
