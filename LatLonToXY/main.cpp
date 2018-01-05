#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <iomanip>
#include <string.h>
#include "Geography/Geography.hpp"
#include "Geography/GaussLocalGeographicCS.hpp"
#include <pthread.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_plotter.h>


using namespace std;

ofstream recordXY("/home/slam/LatLonToXY/Odom/RTK.txt", ios::trunc);
ofstream recordXY_("/home/slam/LatLonToXY/Odom/RTK_.txt", ios::trunc);
ifstream readRTK("/home/slam/LatLonToXY/Odom/thisCarGPS.txt");
Geography::CSTransfer<GaussLocalGeographicCS> csTransfer;
void readRTKFunc();
void writeXYFunc(double x_, double y_, double x, double y);
void rotateGPSCoordinate(double x, double y, double* x_after, double* y_after, double x_diff, double y_diff);


int main(int argc, char** argv) {
	readRTKFunc();
	return 0;
}


void readRTKFunc() {
	double lat = 0, lon = 0;
	double x =0, y = 0;
	double first_x = 0, first_y = 0;
	char ch1 = '0';
	bool isFirstRecord = true;
	if (!readRTK.is_open()) {
		cout << "error opening the file" << endl;
	}
	while(!readRTK.eof()) {
		readRTK >> lon;
		readRTK >> ch1;
		readRTK >> lat;
		csTransfer.ll2xy(lat, lon, x, y);
		if (isFirstRecord == true) {
			first_x = x;
			first_y = y;
			isFirstRecord = false;
		}
		double thisCar_x = 0, thisCar_y = 0;
		rotateGPSCoordinate(x, y, &thisCar_x, &thisCar_y, -50, -50);
		writeXYFunc(thisCar_x, thisCar_y, x, y);
	}
}


void writeXYFunc(double x_, double y_, double x, double y) {
	recordXY << x << setprecision(20) << " " << y << endl;
	recordXY_ << x_ << setprecision(20) << " " << y_ << endl;
}

void rotateGPSCoordinate(double x, double y, double* x_after, double* y_after, double x_diff, double y_diff) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Vector4f GPSPosition = Eigen::Vector4f::Zero();
    float rotate_coordinate = M_PI/2;
    transform(0, 0) = cos(rotate_coordinate);
    transform(0, 1) = -sin(rotate_coordinate);
    transform(0, 2) = 0;
    transform(0, 3) = x_diff;
    transform(1, 0) = sin(rotate_coordinate);
    transform(1, 1) = cos(rotate_coordinate);
    transform(1, 2) = 0;
    transform(1, 3) = y_diff;
    
    GPSPosition(0) = x;
    GPSPosition(1) = y;
    GPSPosition(2) = 0;
    GPSPosition(3) = 1;
    GPSPosition = transform * GPSPosition;
    *x_after = GPSPosition(0);
    *y_after = GPSPosition(1);
}