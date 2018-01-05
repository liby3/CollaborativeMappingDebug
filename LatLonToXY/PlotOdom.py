# -*- coding: utf-8 -*-  
import os
import numpy as np  
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
from mpl_toolkits.mplot3d import Axes3D

fileThisCarRTK = open("Odom/RTK.txt")
fileThisCarRTK_ = open("Odom/RTK_.txt")
fileThisCarOdom = open("Odom/path.txt")

thisCarXRTK = []  
thisCarYRTK = []
thisCarXRTK_ = []  
thisCarYRTK_ = []
thisCarXOdom = []
thisCarYOdom = []
thisCarZOdom = []

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
	lineThisCarRTK_ = fileThisCarRTK_.readline()
	if not lineThisCarRTK_:
		break
	lineThisCarRTK_ = lineThisCarRTK_.split(' ')
	x3 = float(lineThisCarRTK_[0])
	y3 = float(lineThisCarRTK_[1])
	thisCarXRTK_.append(x3)
	thisCarYRTK_.append(y3)
fileThisCarRTK_.close()


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


plt.figure(figsize=(10, 10))
plot1, = plt.plot(thisCarXRTK, thisCarYRTK,"r",linewidth=2)
plot3, = plt.plot(thisCarXRTK_, thisCarYRTK_,"b",linewidth=2)
#plot2, = plt.plot(thisCarXOdom, thisCarZOdom,"b",linewidth=2)
plt.xlabel("x")
plt.ylabel("y")
plt.xlim(-300, 300)
plt.ylim(-300, 300)
#plt.legend([plot1, plot2], ['this car RTK', 'this car LiDAR'], loc='upper right')
plt.legend([plot1, plot3], ['this car RTK', 'this car RTK after rt'], loc='upper right')
plt.title("Map Trajectory")
plt.savefig("map3.png")
plt.show()
