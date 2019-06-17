#!/usr/bin/env python
import actionlib
import rospy
import tf.transformations
import fetch_api
import sys
import moveit_commander
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState

from gazebo_msgs.srv import GetLinkState
import math
from graspit_commander import GraspitCommander
import numpy as np
from gazebo_msgs.srv import GetModelState
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from tf import TransformListener
from tf.transformations import quaternion_from_euler

def remove_table(moveit_commander):
	scene = moveit_commander.PlanningSceneInterface()
	scene.remove_world_object("table")
	rospy.sleep(2)

def remove_target(moveit_commander):
	scene = moveit_commander.PlanningSceneInterface()
	scene.remove_world_object("target")
	rospy.sleep(2)

def gms_client(model_name,relative_entity_name):
	rospy.wait_for_service('/gazebo/get_link_state')
	try:
		gms = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		resp1 = gms(model_name,relative_entity_name)
		return resp1
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def gms_client_model(model_name,relative_entity_name):
	rospy.wait_for_service('/gazebo/get_model_state')
	try:
		gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		resp1 = gms(model_name,relative_entity_name)
		return resp1
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def add_collision_objects( moveit_commander, given_pose=None, attach = False):
	scene = moveit_commander.PlanningSceneInterface()
	rospy.sleep(2)
	print("adding collision objects")
	p1 = geometry_msgs.msg.PoseStamped()
	p1.header.frame_id = "base_link"
	p1.pose = gms_client("table1::chassis","fetch::base_link").link_state.pose
	#p_in_base = tfl.transformPose("/base_link", p1)
	#print("trasnformed_pose -------------------------",p_in_base)
	# obst = scene.makeSolidPrimitive("table","box",ps)
	#ps = transformPoint("base_link",ps.pose.position)
	# obst.setColor("table",1,0,0)
	p1.pose.position.z = 0.3875
	scene.add_box("table",p1,(0.913,0.913,0.775))
	#scene.add_mesh("table",ps,'cafe_table.dae.ply')
	rospy.sleep(2)
	#scene.sendUpdate()

	b = geometry_msgs.msg.PoseStamped()
	b.header.frame_id = "base_link"
	b.pose = gms_client("crackers::link","fetch::base_link").link_state.pose
	b.pose.position.z += 0.105437
	qua = quaternion_from_euler(-1.570794, -0.000001, 2.094129 )#-1.55000)
	b.pose.orientation.x = qua[0]
	b.pose.orientation.y = qua[1]
	b.pose.orientation.z = qua[2]
	b.pose.orientation.w = qua[3]			
	if attach:
		scene.attach_box("l_gripper_finger_link","target", b, (0.164036, 0.213437, 0.071799),['l_gripper_finger_link','r_gripper_finger_link'])
	else:
		scene.add_box("target", b,(0.164036, 0.213437, 0.071799))
	rospy.sleep(2)

def move_arm(pose,moveit_commander,display_trajectory_publisher):
	group_name = "arm"
	group = moveit_commander.MoveGroupCommander(group_name)
	scene = moveit_commander.PlanningSceneInterface()
	robot = moveit_commander.RobotCommander()
	flag=False
	group.set_pose_target(pose)
	plan1 = group.plan()
	if(len(plan1.joint_trajectory.points)!=0):
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = robot.get_current_state()
		display_trajectory.trajectory.append(plan1)
		display_trajectory_publisher.publish(display_trajectory)
		group.execute(plan1, wait=True)
		rospy.sleep(5)
		flag=True
	return flag

def graspit():
	GraspitCommander.clearWorld()
	rotation = tf.transformations.quaternion_from_euler(1.5707963, 0, -2.094129)
	poses = geometry_msgs.msg.Pose()
	poses.position.x = 3
	poses.position.y = 3
	poses.position.z = 0.001
	poses.orientation.x = rotation[0]
	poses.orientation.y = rotation[1]
	poses.orientation.z = rotation[2]
	poses.orientation.w = rotation[3]

	GraspitCommander.importObstacle("floor")
	GraspitCommander.importGraspableBody("cracker",pose=poses)
	GraspitCommander.importRobot("fetch_gripper")
	plan = GraspitCommander.planGrasps(max_steps=50000)
	return plan

def pose_to_T(apose):
	ori = []
	ori.append(apose.pose.orientation.x)
	ori.append(apose.pose.orientation.y)
	ori.append(apose.pose.orientation.z)
	ori.append(apose.pose.orientation.w)

	pos = []
	pos.append(apose.pose.position.x)
	pos.append(apose.pose.position.y)
	pos.append(apose.pose.position.z)
	object_Q = tf.transformations.quaternion_matrix(np.array(ori))
	object_T = tf.transformations.translation_matrix(np.array(pos))
	return np.dot(object_T,object_Q)

def callback(data, args):
	remove_table(moveit_commander)
	remove_target(moveit_commander)
	listener = args
	pub2 = rospy.Publisher('graspit_return', PoseStamped, queue_size=10)
	pub3 = rospy.Publisher('gripper_fetch', PoseStamped, queue_size=10)
	pub4 = rospy.Publisher('grasp_pose', PoseStamped, queue_size=10)
	grasp_plan = graspit()
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=10)
	t = listener.getLatestCommonTime("/head_camera_rgb_optical_frame","/head_tilt_link")
	position, quaternion = listener.lookupTransform("/head_tilt_link","/head_camera_rgb_optical_frame",t)
	object_Q = tf.transformations.quaternion_matrix(np.array(quaternion))
	object_T = tf.transformations.translation_matrix(np.array(position))
	optical_T_tilt = np.dot(object_T,object_Q)
    	relative1_pose = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    	fetch_pose = relative1_pose("fetch::head_tilt_link","fetch::base_link")

 	camera_T_base =  pose_to_T(fetch_pose.link_state)
 	obj_T_camera =  pose_to_T(data)
	obj_T_base = np.dot(np.dot(camera_T_base, optical_T_tilt), obj_T_camera)

	obj_T_base_msg = geometry_msgs.msg.Pose()
	tr = tf.transformations.translation_from_matrix(obj_T_base)
	obj_T_base_msg.position.x = tr[0]
	obj_T_base_msg.position.y = tr[1]
	obj_T_base_msg.position.z = tr[2]
	qua = tf.transformations.quaternion_from_matrix(obj_T_base)
	obj_T_base_msg.orientation.x = qua[0]
	obj_T_base_msg.orientation.y = qua[1]
	obj_T_base_msg.orientation.z = qua[2]
	obj_T_base_msg.orientation.w = qua[3]
	add_collision_objects(moveit_commander)
	for grasp in grasp_plan.grasps:
		grasp.pose.position.x -=3
	 	grasp.pose.position.y -=3
	 	grasp.pose.position.z -=0.001
	
	 	gripper_T_obj = pose_to_T(grasp)

		#while 1

		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "base_link"
		pose.pose = obj_T_base_msg
		pub3.publish(pose)
		base_T_gripper = np.dot(obj_T_base, np.linalg.inv(gripper_T_obj))

		base_T_gripper_msg = geometry_msgs.msg.Pose()
		tr = tf.transformations.translation_from_matrix(base_T_gripper)
		base_T_gripper_msg.position.x = tr[0]
		base_T_gripper_msg.position.y = tr[1]
		base_T_gripper_msg.position.z = tr[2]
		qua = tf.transformations.quaternion_from_matrix(base_T_gripper)
		base_T_gripper_msg.orientation.x = qua[0]
		base_T_gripper_msg.orientation.y = qua[1]
		base_T_gripper_msg.orientation.z = qua[2]
		base_T_gripper_msg.orientation.w = qua[3]

		pose1 = PoseStamped()
		pose1.header.stamp = rospy.Time.now()
		pose1.header.frame_id = "base_link"
		pose1.pose = base_T_gripper_msg
		pub4.publish(pose1)			
		
		status = move_arm(base_T_gripper_msg, moveit_commander, display_trajectory_publisher)
		if status == False:
			continue
		else:
			break
	rospy.sleep(2)
	#gripper.close(55)
	#add_collision_objects(moveit_commander, True)
	#arm.move_to_joints(fetch_api.ArmJoints.from_list([0,0,0,0,0,0,0]))
	remove_table(moveit_commander)
	remove_target(moveit_commander)
	rospy.sleep(2)		

if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node("pick_demo")
	remove_table(moveit_commander)
	remove_target(moveit_commander)
	listener = TransformListener()
	base = fetch_api.Base()
	gripper = fetch_api.Gripper()
	torso = fetch_api.Torso()
	arm = fetch_api.Arm()
	pi=math.pi
	arm.move_to_joints(fetch_api.ArmJoints.from_list([pi/3,-pi/4,0,-pi/4, -pi/3 ,2.1,pi/2]))
	#arm.move_to_joints(fetch_api.ArmJoints.from_list([1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]))
	rospy.sleep(2)
	add_collision_objects(moveit_commander)
	
	#arm.move_to_joints(fetch_api.ArmJoints.from_list([pi/3,-pi/4,0,-pi/4, -pi/3 ,2.1,pi/2]))
	dope_subscriber = rospy.Subscriber('/dope/pose_cracker',PoseStamped,callback, (listener))
	rospy.spin()
 
    

