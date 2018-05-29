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
# And for reading CSV files and plotting data.
import serial
import time
import csv
from datetime import datetime
import matplotlib.pyplot as plt

# Reads CSV files. Input name. Output column time series data.
def ThisIsAFunctionsUsedToReadCSVFiles(fileName_IN):
	# Initialize time series lists.
	controlCycle = []
	angleThumb = []
	angleRefThumb = []
	angleIndex = []
	angleRefIndex = []
	Bx = []
	By = []
	Bz = []
	graspState = []
	motionState = []
	with open(fileName_IN, 'rb') as csvfile:
		datareader = csv.reader(csvfile, delimiter=' ', quotechar='|')

		# Skip first line
		firstLine = True
		for row in datareader:
			if(firstLine):
				firstLine = False
				continue
			newRow = row[0].replace('"',"")
			newRow = (newRow.split(","))
			try:
				controlCycle.append(int(newRow[0]))
				angleThumb.append(float(newRow[1]))
				angleRefThumb.append(float(newRow[2]))
				angleIndex.append(float(newRow[3]))
				angleRefIndex.append(float(newRow[4]))
				Bx.append(int(newRow[5]))
				By.append(int(newRow[6]))
				Bz.append(int(newRow[7]))
				graspState.append(int(newRow[8]))
				motionState.append(int(newRow[9]))

			except ValueError, e:
				print "error",e,"on line", row

	return controlCycle, angleThumb, angleRefThumb, angleIndex, angleRefIndex, Bx, By, Bz, graspState, motionState

if __name__ == '__main__':
	#fileName = "graspExperimentData2018_05_23_18_10_27_Empty_01.csv"
	#fileName = "graspExperimentData2018_05_23_18_10_57_Loaded_01.csv"
	#fileName = "graspExperimentData2018_05_23_18_11_28_Loaded_02.csv"
	#fileName = "graspExperimentData2018_05_23_18_11_58_Loaded_03.csv"
	#fileName = "graspExperimentData2018_05_23_18_12_30_Loaded_04.csv"
	#fileName = "graspExperimentData2018_05_23_18_12_59_Loaded_05.csv"
	#fileName = "graspExperimentData2018_05_23_18_13_31_Empty_02.csv"
	
	#fileName = "graspExperimentData2018_05_25_16_28_38.csv"

	fileName = "graspExperimentData2018_05_28_19_08_42.csv"


	print("Hello World")

	controlCycle_Data, \
	angleThumb_Data, angleRefThumb_Data, \
	angleIndex_Data, angleRefIndex_Data, \
	Bx_Data, By_Data, Bz_Data, \
	graspState_Data, motionState_Data = ThisIsAFunctionsUsedToReadCSVFiles(fileName)

	#print(graspState_Data)
	# X is parallel to gravity. Y ix perpendicular to sensing surface. Z is cross product of X and Y.
	Bz_Data = [i+500 for i in Bz_Data]
	By_Data = [i-150 for i in By_Data]
	Bx_Data = [i+50 for i in Bx_Data]

	timeArray = np.arange(len(controlCycle_Data))

	# Control Rate
	plt.figure(1)
	plt.plot(timeArray, controlCycle_Data, 'k--',linewidth=3)
	plt.grid(True)
	plt.ylabel('Control Rate [A.U.]', fontsize=20)
	plt.xlabel('Time [ms]', fontsize=20)

	# Thumb and Index angles
	plt.figure(2)
	plt.subplot(211)
	plt.plot(timeArray, angleThumb_Data, 'b', timeArray, angleRefThumb_Data, 'r',linewidth=3)
	plt.grid(True)
	plt.title('Thumb')
	plt.ylim((0,90))
	plt.ylabel('Angle [deg]', fontsize=20)

	plt.subplot(212)
	plt.plot(timeArray, angleIndex_Data, 'b', timeArray, angleRefIndex_Data, 'r',linewidth=3)
	plt.grid(True)
	plt.title('Index')
	plt.ylim((0,90))
	plt.ylabel('Angle [deg]', fontsize=20)
	plt.xlabel('Time [ms]', fontsize=20)

	# Magnetic Measurements
	plt.figure(3)
	plt.plot(timeArray, Bx_Data, 'r', timeArray, By_Data, 'g',Bz_Data, 'b',linewidth=3)
	plt.plot(timeArray, [i * 200 for i in graspState_Data],'k', timeArray, [i * 50 for i in motionState_Data],'m',linewidth=3)
	plt.grid(True)
	plt.ylim((-200,500))
	plt.ylabel('Magnetic Field [V]', fontsize=20)
	plt.xlabel('Time [ms]', fontsize=20)	

	# Grasp State
	#plt.figure(4)
	#plt.plot(timeArray, graspState_Data, 'b',linewidth=3)
	#plt.grid(True)
	#plt.ylabel('Grasp State [A.U]', fontsize=20)
	#plt.xlabel('Time [ms]', fontsize=20)	

	# Motion State
	#plt.figure(5)
	#plt.plot(timeArray, motionState_Data, 'b',linewidth=3)
	#plt.grid(True)
	#plt.ylabel('Motion State [A.U]', fontsize=20)
	#plt.xlabel('Time [ms]', fontsize=20)	

	plt.show()



	
