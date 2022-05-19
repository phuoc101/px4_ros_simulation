import cv2
import sys
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import logging

# camera init
camset = 'udpsrc port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink'
# gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
# ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink 
cam = cv2.VideoCapture(camset, cv2.CAP_GSTREAMER)

#!/usr/bin/env python
import rospy 

class DroneVideoStreamNode:
    def __init__(self):
        rospy.init_node("drone_video_stream", log_level=rospy.INFO)
        rospy.loginfo("Starting DroneVideoStreamNode.")
        # image publisher init
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("drone_cam_capture", Image, queue_size=10)
        
        # init timer
        self.rate = rospy.Rate(20)  # in Hz
        # self.timer = rospy.Timer(self.rate, self.timer_callback)
        
    def show_img(self, is_show_img=False):
        global cam
        while not rospy.is_shutdown():
            ret, frame = cam.read()
            if ret:
                img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.img_pub.publish(img_msg)
            else:
                rospy.logwarn("no image")
                sys.exit()
            if is_show_img:
                cv2.imshow('drone cam', frame)
                cv2.waitKey(1)
            self.rate.sleep()

def main(args=None):
    drone_video_stream = DroneVideoStreamNode()
    drone_video_stream.show_img(is_show_img=True)
    rospy.spin()

if __name__ == '__main__':
    main()
