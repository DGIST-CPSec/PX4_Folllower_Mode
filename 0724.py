#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import CommandBoolRequest, SetModeRequest

def state_cb(msg):
    global current_state
    current_state = msg

def takeoff_and_land():
    rospy.wait_for_service('/uav0/mavros/cmd/arming')
    rospy.wait_for_service('/uav0/mavros/set_mode')
    try:
        arming_client = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
        mode_client = rospy.ServiceProxy('/uav0/mavros/set_mode', SetMode)

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for the Flight Controller connection
        while not rospy.is_shutdown() and not current_state.connected:
            rate.sleep()

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2

        # Take off
        for i in range(100):
            if rospy.is_shutdown():
                break

            local_pos_pub.publish(pose)
            rate.sleep()

        # Set to OFFBOARD mode and arm the drone
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if mode_client.call(offb_set_mode).mode_sent == True:
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if arming_client.call(arm_cmd).success == True:
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

            # Keep sending the takeoff pose
            local_pos_pub.publish(pose)

            rate.sleep()

    except rospy.ServiceException as e:
        print("Service call failed:", e)

def main():
    rospy.init_node('takeoff_and_land', anonymous=True)

    global current_state
    current_state = State()

    # Create subscriber for the drone state
    state_subscriber = rospy.Subscriber('/uav0/mavros/state', State, state_cb)

    # Create publisher for the drone
    global local_pos_pub
    local_pos_pub = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    takeoff_and_land()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
