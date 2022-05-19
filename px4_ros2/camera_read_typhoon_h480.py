import cv2
import sys
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import logging

# camera init
camset = 'udpsrc port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink'
# gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
# ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink 
cam = cv2.VideoCapture(camset, cv2.CAP_GSTREAMER)

class DroneImage(Node):
    def __init__(self):
        super().__init__('droneImage')
        # image publisher init
        self.bridge = CvBridge()
        self.imgPub = self.create_publisher(Image, 'drone_cam_capture', 10)
        # init timer
        timerPeriod = 0.01  # in seconds
        self.timer = self.create_timer(timerPeriod, self.timerCallback)

    def timerCallback(self):
        global cam
        # logging.debug("trying to capture image...")
        ret, frame = cam.read()
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # corners, ids, _rejected, = aruco.detectMarkers(gray, aruco_dict)
        # if ids is not None:
        if ret:
            imgMsg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.imgPub.publish(imgMsg)
        else:
            print("no image")
            sys.exit()
        # cv2.imshow('drone cam', frame)
        # cv2.waitKey(1)


def main(args=None):
    logging.basicConfig(format='[%(levelname)s]: %(message)s', level=logging.DEBUG)
    rclpy.init(args=args)
    droneImage = DroneImage()
    rclpy.spin(droneImage)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
            # when the garbage collector destroys the node object)
    droneImage.destroy_node()
    cam.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
