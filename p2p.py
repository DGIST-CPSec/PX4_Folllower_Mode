#!/usr/bin/env python3

import rospy
from threading import Thread
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import CommandBoolRequest, SetModeRequest
import socket
import math

# List of drone namespaces
drone_namespaces = ['uav0', 'uav1', 'uav2', 'uav3']

# Dictionary to store the state of each drone
drone_states = {namespace: State() for namespace in drone_namespaces}

# Dictionary to store the position of 0번 드론
drone_positions = {namespace: PoseStamped() for namespace in drone_namespaces}

def state_cb(namespace, msg):
    drone_states[namespace] = msg

def position_cb(namespace, msg):
    drone_positions[namespace] = msg

def follow_0_and_move(namespace, local_pos_pub, target_drone_namespace):
    rospy.wait_for_service(namespace + '/mavros/cmd/arming')
    rospy.wait_for_service(namespace + '/mavros/set_mode')
    try:
        arming_client = rospy.ServiceProxy(namespace + '/mavros/cmd/arming', CommandBool)
        mode_client = rospy.ServiceProxy(namespace + '/mavros/set_mode', SetMode)

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for the Flight Controller connection
        while not rospy.is_shutdown() and not drone_states[namespace].connected:
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
            if drone_states[namespace].mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if mode_client.call(offb_set_mode).mode_sent == True:
                    rospy.loginfo(namespace + " OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if not drone_states[namespace].armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if arming_client.call(arm_cmd).success == True:
                        rospy.loginfo(namespace + " Vehicle armed")

                    last_req = rospy.Time.now()

            # Move to the position of 0번 드론
            pose = drone_positions[target_drone_namespace]
            local_pos_pub.publish(pose)

            rate.sleep()

    except rospy.ServiceException as e:
        print("Service call failed:", e)

def main():
    rospy.init_node('follow_0_and_move', anonymous=True)

    # Create subscribers for each drone state and position
    state_subscribers = {}
    position_subscribers = {}
    for namespace in drone_namespaces:
        state_subscribers[namespace] = rospy.Subscriber('/' + namespace + '/mavros/state', State, lambda msg, ns=namespace: state_cb(ns, msg))
        position_subscribers[namespace] = rospy.Subscriber('/' + namespace + '/mavros/local_position/pose', PoseStamped, lambda msg, ns=namespace: position_cb(ns, msg))

    # Wait for all drones to connect
    rate = rospy.Rate(10)  # 10 Hz
    while not all(drone_states[namespace].connected for namespace in drone_namespaces):
        rospy.loginfo("Waiting for all drones to connect...")
        rate.sleep()

    rospy.loginfo("All drones connected!")

    # Create publisher for each drone
    local_pos_pubs = {namespace: rospy.Publisher('/' + namespace + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
                      for namespace in drone_namespaces}

    # Takeoff and circle uav0
    follow_thread = Thread(target=follow_0_and_move, args=('uav1', local_pos_pubs['uav1'], 'uav0'))
    follow_thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
