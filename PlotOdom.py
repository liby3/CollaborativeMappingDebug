# -*- coding: utf-8 -*-  
import os
import numpy as np  
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
from mpl_toolkits.mplot3d import Axes3D

fileThisCarRTK = open("Odom/thisCarGPSXY.txt")
fileThisCarOdom = open("Odom/path.txt")


fileOtherCarRTK = open("Odom/GPSXYFromOthers.txt")
fileOtherCarOdom = open("Odom/OdomXYFromOthers.txt")


thisCarXRTK = []  
thisCarYRTK = []
thisCarXOdom = []
thisCarYOdom = []
thisCarZOdom = []

otherCarXRTK = []  
otherCarYRTK = []
otherCarXOdom = []
otherCarYOdom = []
otherCarZOdom = []

while 1:
	lineThisCarRTK = fileThisCarRTK.readline()
	if not lineThisCarRTK:
		break
	lineThisCarRTK = lineThisCarRTK.split(' ')
	x1 = float(lineThisCarRTK[0])
	y1 = float(lineThisCarRTK[1])
	thisCarXRTK.append(x1)
	thisCarYRTK.append(y1)
fileThisCarRTK.close()


while 1:
	lineThisCarOdom = fileThisCarOdom.readline()
	if not lineThisCarOdom:
		break
	lineThisCarOdom = lineThisCarOdom.split(' ')
	x2 = float(lineThisCarOdom[2])
	y2 = float(lineThisCarOdom[3])
	z2 = float(lineThisCarOdom[4])
	thisCarXOdom.append(x2)
	thisCarYOdom.append(y2)
	thisCarZOdom.append(z2)
fileThisCarOdom.close()

while 1:
	lineOtherCarRTK = fileOtherCarRTK.readline()
	if not lineOtherCarRTK:
		break
	lineOtherCarRTK = lineOtherCarRTK.split(' ')
	x3 = float(lineOtherCarRTK[0])
	y3 = float(lineOtherCarRTK[1])
	otherCarXRTK.append(x3)
	otherCarYRTK.append(y3)
fileOtherCarRTK.close()

while 1:
	lineOtherCarOdom = fileOtherCarOdom.readline()
	if not lineOtherCarOdom:
		break
	lineOtherCarOdom = lineOtherCarOdom.split(' ')
	x4 = float(lineOtherCarOdom[0])
	y4 = float(lineOtherCarOdom[1])
	z4 = float(lineOtherCarOdom[2])
	otherCarXOdom.append(x4)
	otherCarYOdom.append(y4)
	otherCarZOdom.append(z4)
fileOtherCarOdom.close()


plt.figure(figsize=(10, 10))
plot1, = plt.plot(thisCarXRTK, thisCarYRTK,"r",linewidth=2)
plot2, = plt.plot(thisCarZOdom, thisCarXOdom,"b",linewidth=2)
plot3, = plt.plot(otherCarZOdom, otherCarXOdom,"g",linewidth=2)
plot4, = plt.plot(otherCarXRTK, otherCarYRTK,"black",linewidth=2)
plt.xlabel("x")
plt.ylabel("y")
plt.xlim(-300, 300)
plt.ylim(-300, 300)
plt.legend([plot1, plot2, plot4, plot3], ['this car RTK', 'this car LiDAR', 'another car RTK', 'another car LiDAR'], loc='upper right')
plt.title("Map Trajectory")
plt.savefig("whole map.png")
plt.show()
