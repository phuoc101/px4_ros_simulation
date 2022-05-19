#!/usr/bin/python3
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
import rospy

current_state = State()
def state_callback(msg):
    global current_state
    current_state = msg

def main():
    global current_state
    rospy.init_node("offb_node")
    rospy.loginfo("Starting offb_node.")

    state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
    pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # the setpoint publishing rate MUST be faster than 2Hz
    rate = rospy.Rate(20.0)

    # wait for FCU connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    #send a few setpoints b4 starting
    i = 100
    while i > 0:
        i -= 1
        pos_pub.publish(pose)
        rate.sleep()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD":
            rospy.loginfo("Trying to enable offboard...")
            rospy.wait_for_service('mavros/set_mode')
            set_mode_response = set_mode_client(custom_mode="OFFBOARD")
            if set_mode_response.mode_sent:
                rospy.loginfo("Offboard enabled")
        elif not current_state.armed:
            rospy.wait_for_service('mavros/cmd/arming')
            arming_response = arming_client(value=True)
            if arming_response.success:
                rospy.loginfo("Vehicle armed")

        pos_pub.publish(pose)
        rate.sleep()

if __name__ == "__main__":
    main()
