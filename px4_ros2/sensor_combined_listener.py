import rclpy
from px4_msgs.msg import SensorCombined
from rclpy.node import Node

class SensorCombinedListener(Node):
    def __init__(self):
        super().__init__('SensorCombinedListener')
        self.sensorCombinedSub = self.create_subscription(SensorCombined, 'fmu/sensor_combined/out', self.sensorCallback, 10)

    def sensorCallback(self, msg):
        print('RECEIVED SENSOR COMBINED DATA')
        print(f"ts: {msg.timestamp}")
        print(f"gyro_rad[0]: {msg.gyro_rad[0]}") 
        print(f"gyro_rad[1]: {msg.gyro_rad[1]}")
        print(f"gyro_rad[2]: {msg.gyro_rad[2]} ")
        print(f"gyro_integral_dt: {msg.gyro_integral_dt}")
        print(f"accelerometer_timestamp_relative: {msg.accelerometer_timestamp_relative}")
        print(f"accelerometer_m_s2[0]: {msg.accelerometer_m_s2[0]}")
        print(f"accelerometer_m_s2[1]: {msg.accelerometer_m_s2[1]} ")
        print(f"accelerometer_m_s2[2]: {msg.accelerometer_m_s2[2]}")
        print(f"accelerometer_integral_dt: {msg.accelerometer_integral_dt}")

def main(args=None):
    rclpy.init(args=args)
    sensorListener = SensorCombinedListener()
    rclpy.spin(sensorListener)
    sensorListener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()