#!/usr/bin/env python
# import necessary library at first
import rospy
import sys
import copy
import tf_conversions
import shape_msgs.msg as shape_msgs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import numpy as np
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from numpy import zeros, array, linspace


# initialize an instance of JointState at first which should be used all over the program
currentJointState = JointState()

def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg

# this function is mainly comes from the example provided by Prof.
def open():
    # to get the current joint state
    currentJointState = rospy.wait_for_message("/joint_states",JointState)
    # set the current time
    currentJointState.header.stamp = rospy.get_rostime()
    tmp = 0.005
    # set the position of each states
    currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
    rate = rospy.Rate(10) # set to 10hz
    for i in range(3):
        publisher_open.publish(currentJointState)
        rate.sleep()
    print'LOOK! The gripper is opened now.'
    
# this function is mainly comes from the example provided by Prof.
def close():
    currentJointState = rospy.wait_for_message("/joint_states",JointState)
    currentJointState.header.stamp = rospy.get_rostime()
    tmp = 0.7
    currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
    rate=rospy.Rate(10)
    for i in range(3):
        publisher_close.publish(currentJointState)
        rate.sleep()
    print 'LOOK! The gripper is closed now.'
# this function is to compute cartesian path
def planning(x_1,y_1,z_1):
    waypoints = list()
    pose_goal = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(pose_goal))
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-math.pi/2, 0))
    pose_goal.position.x = x_1
    pose_goal.position.y = y_1
    pose_goal.position.z = z_1
    waypoints.append(copy.deepcopy(pose_goal))
    fraction = 0.0
    attempts_max = 100
    attempts = 0
    while fraction < 0.6 and attempts < attempts_max:
        (plan1, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        attempts += 1
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
    if fraction >0.4:
        rospy.loginfo(
            "Path planning  " + str(fraction) + " success after " + str(
                attempts_max) + " attempts.")
        rospy.loginfo("Path computed successfully. Moving the arm.")
        group.execute(plan1)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo(
            "Path planning failed with only " + str(fraction) + " success after " + str(
                attempts_max) + " attempts.")
        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal)
        plan1 = group.plan()
        group.execute(plan1)
# Initialization
moveit_commander.roscpp_initialize(sys.argv)
# create the publisher of open and close operation respectively
publisher_open = rospy.Publisher('/jaco/joint_control',JointState,queue_size=1)
publisher_close = rospy.Publisher('/jaco/joint_control',JointState,queue_size=1)
# create a ros node in this file note that only one node can be created by each file
rospy.init_node('move_the_cubes',anonymous=True)
# These three codes are all the default settings which we should get the instance of each Class
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("Arm")

# Setup the planner
group.set_goal_orientation_tolerance(0.01)
group.set_goal_tolerance(0.01)
group.set_goal_joint_tolerance(0.05)
group.set_num_planning_attempts(150)

# Get the position of the cubes
models = rospy.wait_for_message('/gazebo/model_states',ModelStates)
poses = models.pose
model_number = len(poses)
# We want to see how many models in our application
print'Now we have ' + str(model_number-3) + ' cubes.'
# move each cube
cube_number = model_number - 3;
pose_goal = group.get_current_pose().pose
init_pose = pose_goal
# Let the hand toward to table.
print'The hand is facing to table'
hand_to_table = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-math.pi/2, 0))
pose_goal = group.get_current_pose().pose
pose_goal.orientation = hand_to_table
group.set_start_state_to_current_state()
group.set_pose_target(pose_goal)
plan_s = group.plan()
group.execute(plan_s)
for i in range(cube_number):
    #To get the postion of cube since the previous 2 models are desk and robot
    start_idx = 2 + i
    ori = poses[start_idx].orientation
    pos = poses[start_idx].position
    print'orientation is: '
    print(ori)
    print'position is: '
    print(pos)
    #To get the position of bucket which is the target position in our application
    bucket_pose = poses[model_number-1]
    bucket_position = bucket_pose.position

    base_x = 0.5788
    base_y = -0.0154
    dist_threshold_far = 0.55
    dist_threshold_near = 0.25
    dist = np.sqrt(np.square(pos.x - base_x) + np.square(pos.y - base_y))
    if dist < dist_threshold_far and dist > dist_threshold_near:
	    print'Distence is: '
	    print(dist)
	    ## Now let us start moving these cubes to the target position
	    # 1.step1: move the robot to current cube
	    # Bring the arm to the location of the object, minus a certain amount of space
	    print'Firstly, move the robot to the top of the cube'
	    planning(pos.x,pos.y,pos.z + 0.25)
	    rospy.sleep(1)
	    #2.step2: open the gripper
	    print'Secondly, open the gripper'
	    open()
	    rospy.sleep(1)
	    #3.step3: move the manipulator downward a bit to approach the cube
	    print'Thirdly, move manipulator downward and approach the cube'
	    planning(pos.x,pos.y,pos.z + 0.2)
	    rospy.sleep(1)

	    #4.step4: close the gripper
	    print'Close the fucking gripper'
	    close()
	    rospy.sleep(3)

	    #5.step5: raise the manipulator
	    print'Raise the manipulator'
	    planning(pos.x,pos.y,pos.z + 0.5)
	    rospy.sleep(1)

	    #6.step6: move to the top of bucket
	    planning(bucket_position.x,bucket_position.y,1.35)
	    rospy.sleep(1)   
	    #7.step7: release the cube
	    print'Release the cube'
	    open()
	    rospy.sleep(3)
	    #8.step8: Back to init position
    else :
    	print'Out of range can not get'
    	print'Distence is: '
    	print(dist)
    	continue
close()
rospy.sleep(5)
print'back'
group.set_pose_target(init_pose)
plan_e = group.plan()
group.go(wait=True)
print'back finish'
rospy.sleep(1)
group.stop()
