#! /usr/bin/env python

import math
import fetch_api
import rospy


if __name__ == '__main__':
    rospy.init_node('motion_demo')

    base = fetch_api.Base()
    head = fetch_api.Head()
    torso = fetch_api.Torso()
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    print('start')
    head.pan_tilt(0, math.pi/4)
    torso.set_height(0.4)
    pi=math.pi
    arm.move_to_joints(fetch_api.ArmJoints.from_list([1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]))
    rospy.sleep(2)
    arm.move_to_joints(fetch_api.ArmJoints.from_list([pi/3,-pi/4,0,-pi/4, -pi/3 ,2.1,pi/2]))

    # pi=math.pi
    # arm.move_to_joints(fetch_api.ArmJoints.from_list([1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]))
    # rospy.sleep(2)
    # arm.move_to_joints(fetch_api.ArmJoints.from_list([pi/3,-pi/4,0,-pi/4, -pi/3 ,2.1,pi/2]))
    # base.go_forward(0.75)
    # arm.move_to_joints(fetch_api.ArmJoints.from_list([0,0,0,0,0,0,0]))
    # print('arm finished')
    # gripper.open()
    # gripper.close()
    # print('gripper finished')
