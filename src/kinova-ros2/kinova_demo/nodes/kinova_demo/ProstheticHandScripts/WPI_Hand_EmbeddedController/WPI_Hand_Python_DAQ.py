#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys
import numpy as np

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg


import math
import argparse

# Libraries added for Arduino Due Communication. Selim. 2018/05
import serial
import time
import csv
from datetime import datetime

""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq

motionStateCounter = 0

def cartesian_pose_client(position, orientation):
	"""Send a cartesian goal to the action server."""
	action_address = '/' + prefix + 'driver/pose_action/tool_pose'
	client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
	client.wait_for_server()

	goal = kinova_msgs.msg.ArmPoseGoal()
	goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
	goal.pose.pose.position = geometry_msgs.msg.Point(
		x=position[0], y=position[1], z=position[2])
	goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
		x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

	# print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

	client.send_goal(goal, done_cb = doneCB)

	#if client.wait_for_result(rospy.Duration(10.0)):
	#	return client.get_result()
	#else:
	#	client.cancel_all_goals()
	#	print('        the cartesian action timed-out')
	#	return None


def QuaternionNorm(Q_raw):
	qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
	qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
	qx_ = qx_temp/qnorm
	qy_ = qy_temp/qnorm
	qz_ = qz_temp/qnorm
	qw_ = qw_temp/qnorm
	Q_normed_ = [qx_, qy_, qz_, qw_]
	return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
	Q_normed = QuaternionNorm(Q_raw)
	qx_ = Q_normed[0]
	qy_ = Q_normed[1]
	qz_ = Q_normed[2]
	qw_ = Q_normed[3]

	tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
	ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
	tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
	EulerXYZ_ = [tx_,ty_,tz_]
	return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
	tx_, ty_, tz_ = EulerXYZ_[0:3]
	sx = math.sin(0.5 * tx_)
	cx = math.cos(0.5 * tx_)
	sy = math.sin(0.5 * ty_)
	cy = math.cos(0.5 * ty_)
	sz = math.sin(0.5 * tz_)
	cz = math.cos(0.5 * tz_)

	qx_ = sx * cy * cz + cx * sy * sz
	qy_ = -sx * cy * sz + cx * sy * cz
	qz_ = sx * sy * cz + cx * cy * sz
	qw_ = -sx * sy * sz + cx * cy * cz

	Q_ = [qx_, qy_, qz_, qw_]
	return Q_

def doneCB(self, goal):
	global motionStateCounter
	print("Callback Entered")

	# Don't increment here on certain states
	if(motionStateCounter != 3 and motionStateCounter != 7):
		motionStateCounter = motionStateCounter + 1
	print "Motion State:", motionStateCounter
	

def getcurrentCartesianCommand(prefix_):
	# wait to get current position
	topic_address = '/' + prefix_ + 'driver/out/cartesian_command'
	rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, setcurrentCartesianCommand)
	rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
	print 'position listener obtained message for Cartesian pose. '


def setcurrentCartesianCommand(feedback):
	global currentCartesianCommand

	currentCartesianCommand_str_list = str(feedback).split("\n")

	for index in range(0,len(currentCartesianCommand_str_list)):
		temp_str=currentCartesianCommand_str_list[index].split(": ")
		currentCartesianCommand[index] = float(temp_str[1])
	# the following directly reading only read once and didn't update the value.
	# currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z] 
	# print 'currentCartesianCommand in setcurrentCartesianCommand is: ', currentCartesianCommand


def argumentParser(argument_):
	""" Argument parser """
	parser = argparse.ArgumentParser(description='Drive robot end-effector to command Cartesian pose')
	parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
						help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
	parser.add_argument('unit', metavar='unit', type=str, nargs='?', default='mq',
						choices={'mq', 'mdeg', 'mrad'},
						help='Unit of Cartesian pose command, in mq(Position meter, Orientation Quaternion),  mdeg(Position meter, Orientation Euler-XYZ in degree), mrad(Position meter, Orientation Euler-XYZ in radian)]')
	parser.add_argument('pose_value', nargs='*', type=float, help='Cartesian pose values: first three values for position, and last three(unit mdeg or mrad)/four(unit mq) for Orientation')
	parser.add_argument('-r', '--relative', action='store_true',
						help='the input values are relative values to current position.')
	parser.add_argument('-v', '--verbose', action='store_true',
						help='display Cartesian pose values in alternative convention(mq, mdeg or mrad)')
	# parser.add_argument('-f', action='store_true', help='assign finger values from a file')

	args_ = parser.parse_args(argument_)
	# print('pose_mq in argumentParser 1: {}'.format(args_.pose_value))  # debug
	return args_


def kinova_robotTypeParser(kinova_robotType_):
	""" Argument kinova_robotType """
	global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
	robot_category = kinova_robotType_[0]
	robot_category_version = int(kinova_robotType_[1])
	wrist_type = kinova_robotType_[2]
	arm_joint_number = int(kinova_robotType_[3])
	robot_mode = kinova_robotType_[4]
	finger_number = int(kinova_robotType_[5])
	prefix = kinova_robotType_ + "_"
	finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
	finger_maxTurn = 6800  # max thread turn for one finger


def unitParser(unit_, pose_value_, relative_):
	""" Argument unit """
	global currentCartesianCommand

	position_ = pose_value_[:3]
	orientation_ = pose_value_[3:]

	for i in range(0,3):
		if relative_:
			position_[i] = pose_value_[i] + currentCartesianCommand[i]
		else:
			position_[i] = pose_value_[i]

	# print('pose_value_ in unitParser 1: {}'.format(pose_value_))  # debug

	if unit_ == 'mq':
		if relative_:
			orientation_XYZ = Quaternion2EulerXYZ(orientation_)
			orientation_xyz_list = [orientation_XYZ[i] + currentCartesianCommand[3+i] for i in range(0,3)]
			orientation_q = EulerXYZ2Quaternion(orientation_xyz_list)
		else:
			orientation_q = orientation_

		orientation_rad = Quaternion2EulerXYZ(orientation_q)
		orientation_deg = list(map(math.degrees, orientation_rad))

	elif unit_ == 'mdeg':
		if relative_:
			orientation_deg_list = list(map(math.degrees, currentCartesianCommand[3:]))
			orientation_deg = [orientation_[i] + orientation_deg_list[i] for i in range(0,3)]
		else:
			orientation_deg = orientation_

		orientation_rad = list(map(math.radians, orientation_deg))
		orientation_q = EulerXYZ2Quaternion(orientation_rad)

	elif unit_ == 'mrad':
		if relative_:
			orientation_rad_list =  currentCartesianCommand[3:]
			orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)]
		else:
			orientation_rad = orientation_

		orientation_deg = list(map(math.degrees, orientation_rad))
		orientation_q = EulerXYZ2Quaternion(orientation_rad)

	else:
		raise Exception("Cartesian value have to be in unit: mq, mdeg or mrad")

	pose_mq_ = position_ + orientation_q
	pose_mdeg_ = position_ + orientation_deg
	pose_mrad_ = position_ + orientation_rad

	# print('pose_mq in unitParser 1: {}'.format(pose_mq_))  # debug

	return pose_mq_, pose_mdeg_, pose_mrad_


def verboseParser(verbose, pose_mq_):
	""" Argument verbose """
	position_ = pose_mq_[:3]
	orientation_q = pose_mq_[3:]
	if verbose:
		orientation_rad = Quaternion2EulerXYZ(orientation_q)
		orientation_deg = list(map(math.degrees, orientation_rad))
		print('Cartesian position is: {}'.format(position_))
		print('Cartesian orientation in Quaternion is: ')
		print('qx {:0.3f}, qy {:0.3f}, qz {:0.3f}, qw {:0.3f}'.format(orientation_q[0], orientation_q[1], orientation_q[2], orientation_q[3]))
		print('Cartesian orientation in Euler-XYZ(radian) is: ')
		print('tx {:0.3f}, ty {:0.3f}, tz {:0.3f}'.format(orientation_rad[0], orientation_rad[1], orientation_rad[2]))
		print('Cartesian orientation in Euler-XYZ(degree) is: ')
		print('tx {:3.1f}, ty {:3.1f}, tz {:3.1f}'.format(orientation_deg[0], orientation_deg[1], orientation_deg[2]))

def SerialInputParser(msgLine):
	msgLine = msgLine[:-2]
	return (msgLine.split(" "))
	#print("Serial Input Parser Entered")

if __name__ == '__main__':
	# Not sure what these are doing. Selim. 05/2018
	args = argumentParser(None)
	kinova_robotTypeParser(args.kinova_robotType)
	rospy.init_node(prefix + 'pose_action_client')
	getcurrentCartesianCommand(prefix)
	print("Hello World")

	action_address = '/' + prefix + 'driver/pose_action/tool_pose'
	client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
	client.wait_for_server()
	
	# Sets some 6D poses for grasp experiments.
	# I'm not quite sure how it arranges 3D orientation.
	# Cartesian positions seem to be working fine.

	# Lift/Init Pose
	positionWP_Init = [0.57, 0.05, 0.3, -0.0, 89.1, -160.4]

	# Pick Pose
	positionWP_Pick = [0.57, -0.1, 0.1, -0.0, 89.1, -160.4]

	# A waypoint over pick position
	positionWP_Pick_Over = [0.57, -0.1, 0.3, -0.0, 89.1, -160.4]

	# Place Pose
	positionWP_Place = [0.57, 0.2, 0.1, -0.0, 89.1, -160.4]

	# A waypoint over place position
	positionWP_Place_Over = [0.57, 0.2, 0.3, -0.0, 89.1, -160.4]

	# Turn off relative movement of the tip in cartesian mode.
	relativeFlag = False

	# Open CSV file for writing experiment data
	fileName = 'graspExperimentData' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv'
	with open(fileName, 'w') as myfile:
		wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
		wr.writerow(["Control Cycle","Angle Thumb","Angle Ref Thumb", "Angle Index", "Angle Ref Index", "Bx", "By", "Bz", "Grasp State", "MotionState"])

		# Open serial com to Arduino due. Make sure baud rate matches with the embedded program.
		ser = serial.Serial('/dev/ttyACM0', 4000000, timeout = 1)
		endTime = 1.0

		# Capture data until end time. This is a sanity check to see hand is working. Data sample should be 1000 samples/sec.
		startTime = time.time()
		dataCount = 0
		while(time.time() - startTime < endTime):
			
			ser.write('b')
			inputLine = ser.readline() 
			print(inputLine)
			parsedInputLine = SerialInputParser(inputLine)
			print(parsedInputLine[8])
			wr.writerow([parsedInputLine[0],parsedInputLine[1],parsedInputLine[2],parsedInputLine[3],parsedInputLine[4],parsedInputLine[5],parsedInputLine[6],parsedInputLine[7],parsedInputLine[8], 0])
			dataCount += 1

		# Close serial communication to Arduino Due
		# Make sure to close right after comm ends. Otherwise send buffer @Due side gets full and blocks control software. Selim. 05/2018
		ser.close()     

		print "Number of captured data points during initialization is", dataCount , "in", endTime, "seconds."

		# Experiment loop starts here
		# Serial opened for capturing experiment data. No time limit. Works until Jaco motion ends.
		motionEndFlag = 0
		motionStateCounter_Prev = -1
		ser = serial.Serial('/dev/ttyACM0', 4000000, timeout = 1)
		ser.readline()
		handCloseStartTime = 0
		handOpenStartTime = 0
		handCloseEndTime = 2.0
		handOpenEndTime = 2.0
		posList = positionWP_Init
		
		experimentStartTime = time.time()
		experimentDataCounter = 0
		while(motionEndFlag == 0):
		#while ( False ):
			# Make sure you always read from Due. Otherwise buffer gets messed up.
			if(ser.isOpen() == False and motionStateCounter > 0):
				ser = serial.Serial('/dev/ttyACM0', 4000000, timeout = 1)
				ser.write('b')
			if(ser.isOpen() == True and motionStateCounter > 0):
				inputLine = ser.readline()
				parsedInputLine = SerialInputParser(inputLine)
				experimentDataCounter = experimentDataCounter + 1
				wr.writerow([parsedInputLine[0],parsedInputLine[1],parsedInputLine[2],parsedInputLine[3],parsedInputLine[4],parsedInputLine[5],parsedInputLine[6],parsedInputLine[7],parsedInputLine[8],motionStateCounter])

			# If motion state changes do the following. Motion state changes are triggered by the Done Callback of ROS.
			if(motionStateCounter_Prev != motionStateCounter):

				motionStateCounter_Prev = motionStateCounter

				# Init experiment
				if(motionStateCounter == 0):
					posList = positionWP_Init

				# Go to pickup position
				if(motionStateCounter == 1):
					posList = positionWP_Pick_Over
					
				# Close Hand
				if(motionStateCounter == 2):
					posList = positionWP_Pick

				# Lift it up
				if(motionStateCounter == 3):
					ser.write('a')
					handCloseStartTime = time.time()
					print ("Closing Hand")

				# Go to place position
				if(motionStateCounter == 4):
					posList = positionWP_Init

				# Open the hand
				if(motionStateCounter == 5):
					posList = positionWP_Place_Over

				# Go back to init position
				if(motionStateCounter == 6):
					posList = positionWP_Place

				# Go back to init position
				if(motionStateCounter == 7):
					ser.write('b')
					handOpenStartTime = time.time()
					print ("Opening Hand")

				# Go back to init position
				if(motionStateCounter == 8):
					posList = positionWP_Init
					motionEndFlag = 1										

				#posList = positionWP_Pick
				pose_mq, pose_mdeg, pose_mrad = unitParser(args.unit, posList, relativeFlag)
				try:

					poses = [float(n) for n in pose_mq]
					result = cartesian_pose_client(poses[:3], poses[3:])

					print('Cartesian pose sent!')

				except rospy.ROSInterruptException:
					print "program interrupted before completion"


			#print (time.time() - handCloseStartTime)
			if(motionStateCounter == 3):
				print (time.time() - handCloseStartTime)

			if( motionStateCounter == 3 and (time.time() - handCloseStartTime > handCloseEndTime)):
				motionStateCounter = motionStateCounter + 1
				print "Motion State:", motionStateCounter

			#if(motionStateCounter == 7):
				#print (time.time() - handOpenStartTime)

			if( motionStateCounter == 7 and (time.time() - handOpenStartTime > handOpenEndTime)):
				motionStateCounter = motionStateCounter + 1
				print "Motion State:", motionStateCounter

			# This is used to detect change in shear to make a more gentle place operation.
			if(motionStateCounter == 6):

				Bz_currentCycle = float(parsedInputLine[7]) +500.0
				print(Bz_currentCycle)
				if(Bz_currentCycle < 25.0):
					ser.write('b')
					motionStateCounter = 8
					client.cancel_all_goals()

		# Continue capturing data after motions end.
		dataCaptureAfterMotion = time.time()
		dataCaptureAfterMotionEndTime = 2.0

		while (time.time()-dataCaptureAfterMotion < dataCaptureAfterMotionEndTime):
			inputLine = ser.readline()
			parsedInputLine = SerialInputParser(inputLine)
			experimentDataCounter = experimentDataCounter + 1
			wr.writerow([parsedInputLine[0],parsedInputLine[1],parsedInputLine[2],parsedInputLine[3],parsedInputLine[4],parsedInputLine[5],parsedInputLine[6],parsedInputLine[7],parsedInputLine[8],motionStateCounter])


		# Close port as soon as data reading is done so you don't fill the output buffer at Due.
		print("Port Closed")
		ser.close()   

		experimentEndTime = time.time()
		print "Number of captured data points during experiment is", experimentDataCounter , "in", experimentEndTime-experimentStartTime, "seconds."

		verboseParser(args.verbose, poses)		 
  
