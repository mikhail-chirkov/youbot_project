#!/usr/bin/env python
import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_commander
import geometry_msgs
import moveit_msgs.msg
import time
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


def listener():

	speedpub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
	time.sleep(2)
	speed = Twist()
	speed.linear.y = -0.01
	speedpub.publish(speed)
	start = rospy.get_rostime().to_sec()

	while not rospy.is_shutdown():
		diff = rospy.get_rostime().to_sec() - start
		if (diff >= 10):
			speed.linear.y = 0
			speedpub.publish(speed)
			time.sleep(2)
			break
		rospy.loginfo(diff)


def moveToTravel(group):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 2.97
	joint_goal[1] = 0.24
	joint_goal[2] = -0.98
	joint_goal[3] = 3.18
	joint_goal[4] = 2.95

	group.go(joint_goal, wait=True)
	group.stop()
	group.clear_pose_targets()

def moveToPreGrasp1(group):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 2.97
	joint_goal[1] = 1.68
	joint_goal[2] = -1.35
	joint_goal[3] = 3.18
	joint_goal[4] = 2.95

	group.go(joint_goal, wait=True)
	group.stop()
	group.clear_pose_targets()

def moveToPostGrasp1(group):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 0
	joint_goal[1] = 1.73
	joint_goal[2] = -1.71
	joint_goal[3] = 3.47
	joint_goal[4] = 2.95

	group.go(joint_goal, wait=True)
	group.stop()
	group.clear_pose_targets()

def moveToXYZ(rgroup,x,y,z,rotx,roty,rotz,w):
	oldPose = rgroup.get_current_pose().pose
	pose_target = geometry_msgs.msg.Pose()
	pose_target.position.x = oldPose.position.x + x
	pose_target.position.y = oldPose.position.y + y
	pose_target.position.z = oldPose.position.z + z
	pose_target.orientation.x = oldPose.orientation.x + rotx
	pose_target.orientation.y = oldPose.orientation.y + roty
	pose_target.orientation.z = oldPose.orientation.z + rotz
	pose_target.orientation.w = oldPose.orientation.w + w
	rgroup.set_pose_target(pose_target)

	plan = rgroup.go(wait=True)
	rgroup.stop()
	rgroup.clear_pose_targets()

def moveToGrasp1(group):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 2.97
	joint_goal[1] = 1.77
	joint_goal[2] = -1.19
	joint_goal[3] = 2.92
	joint_goal[4] = 2.97

	group.go(joint_goal, wait=True)
	group.stop()
	group.clear_pose_targets()

def openGripper(group):
	rospy.loginfo('open')

	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 0.01
	joint_goal[1] = 0.01
	group.go(joint_goal, wait=True)
	group.stop()
	group.clear_pose_targets()


def closeGripper(group):
	rospy.loginfo('close')
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 0.003
	joint_goal[1] = 0.003

	group.go(joint_goal, wait=True)
	group.stop()
	group.clear_pose_targets()

def firstPickPlace():


	arm_group = moveit_commander.MoveGroupCommander("arm")
	gripper_group = moveit_commander.MoveGroupCommander("gripper")
	arm_group.set_planning_time(10)

	robot1_client = actionlib.SimpleActionClient('execute_trajectory',moveit_msgs.msg.ExecuteTrajectoryAction)
	robot1_client.wait_for_server()
	rospy.loginfo('Execute Trajectory server is available for robot1')

	moveToTravel(arm_group)
	time.sleep(1)
	moveToPreGrasp1(arm_group)
	time.sleep(1)
	openGripper(gripper_group)
	time.sleep(1)
	moveToXYZ(arm_group,0.003,0,-0.0385,0,0,0,0)
	closeGripper(gripper_group)
	moveToPreGrasp1(arm_group)
	moveToPostGrasp1(arm_group)
	moveToXYZ(arm_group, 0, 0, -0.024, 0, 0, 0, 0)
	openGripper(gripper_group)
	time.sleep(1)
	moveToPostGrasp1(arm_group)
	time.sleep(1)
	moveToTravel(arm_group)
	time.sleep(1)
	moveToPostGrasp1(arm_group)
	time.sleep(1)
	openGripper(gripper_group)
	moveToXYZ(arm_group, 0, 0, -0.024, 0, 0, 0, 0)
	closeGripper(gripper_group)
	moveToPostGrasp1(arm_group)
	moveToPreGrasp1(arm_group)
	moveToXYZ(arm_group, 0.003, 0, -0.0385, 0, 0, 0, 0)
	openGripper(gripper_group)
	time.sleep(1)
	moveToPreGrasp1(arm_group)
	moveToTravel(arm_group)

def secondPick():

	arm_group = moveit_commander.MoveGroupCommander("arm")
	gripper_group = moveit_commander.MoveGroupCommander("gripper")
	arm_group.set_planning_time(10)

	robot1_client = actionlib.SimpleActionClient('execute_trajectory',moveit_msgs.msg.ExecuteTrajectoryAction)
	robot1_client.wait_for_server()
	rospy.loginfo('Execute Trajectory server is available for robot1')

	moveToTravel(arm_group)
	time.sleep(1)
	moveToPreGrasp1(arm_group)
	time.sleep(1)
	openGripper(gripper_group)
	time.sleep(1)
	moveToXYZ(arm_group,0.003,0,-0.0385,0,0,0,0)
	closeGripper(gripper_group)
	moveToPreGrasp1(arm_group)
	moveToPostGrasp1(arm_group)
	moveToXYZ(arm_group, 0, 0, -0.024, 0, 0, 0, 0)
	openGripper(gripper_group)
	time.sleep(1)
	moveToPostGrasp1(arm_group)
	time.sleep(1)
	moveToTravel(arm_group)
	time.sleep(1)


def secondPlace():

	arm_group = moveit_commander.MoveGroupCommander("arm")
	gripper_group = moveit_commander.MoveGroupCommander("gripper")
	arm_group.set_planning_time(10)

	robot1_client = actionlib.SimpleActionClient('execute_trajectory',moveit_msgs.msg.ExecuteTrajectoryAction)
	robot1_client.wait_for_server()
	rospy.loginfo('Execute Trajectory server is available for robot1')

	moveToPostGrasp1(arm_group)
	time.sleep(1)
	openGripper(gripper_group)
	moveToXYZ(arm_group, 0, 0, -0.024, 0, 0, 0, 0)
	closeGripper(gripper_group)
	moveToPostGrasp1(arm_group)
	moveToPreGrasp1(arm_group)
	moveToXYZ(arm_group, 0.003, 0, -0.0385, 0, 0, 0, 0)
	openGripper(gripper_group)
	time.sleep(1)
	moveToPreGrasp1(arm_group)
	moveToTravel(arm_group)
	rospy.loginfo('Finished!')

def nav_goal_1:
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -1
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
		rospy.logerr("Reached")
		return client.get_result()

def nav_goal_2():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.08
    goal.target_pose.pose.position.y = -0.04
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
		rospy.logerr("Reached")
		return client.get_result()

def attach_handler():
	try:
		s = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
		s.wait_for_service()
		req = AttachRequest()
		req.model_name_1 = "nut"
		req.link_name_1 = "link_2"
		req.model_name_2 = "youbot"
		req.link_name_2 = "base_footprint"

		s.call(req)
	except rospy.ServiceException as e:
		rospy.loginfo("Service failed")

def detach_handler():
	try:
		s = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
		s.wait_for_service()
		req = AttachRequest()
		req.model_name_1 = "nut"
		req.link_name_1 = "link_2"
		req.model_name_2 = "youbot"
		req.link_name_2 = "base_footprint"

		s.call(req)
	except rospy.ServiceException as e:
		rospy.loginfo("Service failed")

if __name__ == '__main__':
	try:
		rospy.init_node('simple_move', anonymous=True)
		moveit_commander.roscpp_initialize(sys.argv)
		firstPickPlace()
		time.sleep(1)
		listener()
		time.sleep(1)
		secondPick()
		time.sleep(1)
		attach_handler()
		time.sleep(1)
		speedpub = rospy.Publisher("cmd_vel", Twist, queue_size=2)
		time.sleep(2)
		speed = Twist()
		result = nav_goal_1()
		if result:
			speedpub.publish(speed)
			time.sleep(2)
			rospy.loginfo("Goal execution done!")
			result = nav_goal_2()
			if result:
				speedpub.publish(speed)
				time.sleep(0.1)
				speedpub.publish(speed)
				time.sleep(2)
				rospy.loginfo("Goal execution done!")
				detach_handler()
				time.sleep(1)
				secondPlace()
		rospy.loginfo('Finished!')
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")