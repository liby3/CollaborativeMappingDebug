#include <iostream>
#include <iomanip>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
 
using namespace std;

void print4x4Matrix(const Eigen::Matrix4f & matrix) {
	std::cout << setw(10) << matrix(0,0) << setw(10) << matrix(0,1) << setw(10) << matrix(0,2) << setw(10) << matrix(0,3) << std::endl;
	std::cout << setw(10) << matrix(1,0) << setw(10) << matrix(1,1) << setw(10) << matrix(1,2) << setw(10) << matrix(1,3) << std::endl;
	std::cout << setw(10) << matrix(2,0) << setw(10) << matrix(2,1) << setw(10) << matrix(2,2) << setw(10) << matrix(2,3) << std::endl;
	std::cout << setw(10) << matrix(3,0) << setw(10) << matrix(3,1) << setw(10) << matrix(3,2) << setw(10) << matrix(3,3) << std::endl;
}

int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloudA(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloudB(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloudA_SVD(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloudA_LM(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloudA_DQ(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloudTemp(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr mapA(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr mapB(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr mapA_SVD(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr mapA_LM(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr mapA_DQ(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::io::loadPCDFile("PCD3/Surf-1.pcd", *pointcloudA);
	pcl::io::loadPCDFile("PCD2/Surf-223.pcd", *pointcloudB);
	pcl::io::loadPCDFile("PCD3/output.pcd", *mapA);
	pcl::io::loadPCDFile("PCD2/output.pcd", *mapB);
	Eigen::Matrix4f transformation_DQ = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation_LM = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation_SVD = Eigen::Matrix4f::Identity();
	pcl::registration::TransformationEstimationSVD< pcl::PointXYZI, pcl::PointXYZI> estSVD;
	pcl::registration::TransformationEstimationLM< pcl::PointXYZI, pcl::PointXYZI> estLM;
	pcl::registration::TransformationEstimationDualQuaternion< pcl::PointXYZI, pcl::PointXYZI> estDQ;
	/* According to the documents, pcl::registration method: source cloud and target cloud must have
     * the same size. As a reuslt, we choose the two cloud to match, and make them the same size.
	 */
	if (pointcloudA->points.size() > pointcloudB->points.size()) {
		for (int i = 0; i < pointcloudB->points.size(); i++) {
			pcl::PointXYZI p;
			p.x = pointcloudA->points[i].x;
			p.y = pointcloudA->points[i].y;
			p.z = pointcloudA->points[i].z;
			p.intensity = pointcloudA->points[i].intensity;
			pointcloudTemp->push_back(p);
		}
		std::cout << pointcloudTemp->points.size() << ", " << pointcloudB->points.size() << std::endl;
		estSVD.estimateRigidTransformation(*pointcloudTemp, *pointcloudB, transformation_SVD);
		estLM.estimateRigidTransformation(*pointcloudTemp, *pointcloudB, transformation_LM);
		estDQ.estimateRigidTransformation(*pointcloudTemp, *pointcloudB, transformation_DQ);
	}
	else {
		for (int i = 0; i < pointcloudA->points.size(); i++) {
			pcl::PointXYZI p;
			p.x = pointcloudB->points[i].x;
			p.y = pointcloudB->points[i].y;
			p.z = pointcloudB->points[i].z;
			p.intensity = pointcloudB->points[i].intensity;
			pointcloudTemp->push_back(p);
		}
		estSVD.estimateRigidTransformation(*pointcloudA, *pointcloudTemp, transformation_SVD);
		estLM.estimateRigidTransformation(*pointcloudA, *pointcloudTemp, transformation_LM);
		estDQ.estimateRigidTransformation(*pointcloudA, *pointcloudTemp, transformation_DQ);
	}
    
    std::cout << "**********************************************************" << std::endl;
    std::cout << "TransformationEstimationSVD: " << std::endl;
    print4x4Matrix(transformation_SVD);
    std::cout << "**********************************************************" << std::endl;
    std::cout << "TransformationEstimationLM: " << std::endl;
    print4x4Matrix(transformation_LM);
    std::cout << "**********************************************************" << std::endl;
    std::cout << "TransformationEstimationDQ: " << std::endl;
    print4x4Matrix(transformation_DQ);
    pcl::transformPointCloud(*pointcloudA, *pointcloudA_SVD, transformation_SVD);
    pcl::transformPointCloud(*pointcloudA, *pointcloudA_LM, transformation_LM);
    pcl::transformPointCloud(*pointcloudA, *pointcloudA_DQ, transformation_DQ);
    pcl::transformPointCloud(*mapA, *mapA_SVD, transformation_SVD);
    pcl::transformPointCloud(*mapA, *mapA_LM, transformation_LM);
    pcl::transformPointCloud(*mapA, *mapA_DQ, transformation_DQ);
	

	PointCloud::Ptr MergePointCloud(new PointCloud);
	for (int j = 0; j < pointcloudA->points.size(); j += 1) {
		pcl::PointXYZRGB p;
		p.x = pointcloudA->points[j].x;
		p.y = pointcloudA->points[j].y;
		p.z = pointcloudA->points[j].z;
		p.r = 255;
		p.g = 0;
		p.b = 0;
		MergePointCloud->points.push_back(p);
	}
	for (int j = 0; j < pointcloudB->points.size(); j += 1) {
		pcl::PointXYZRGB p;
		p.x = pointcloudB->points[j].x;
		p.y = pointcloudB->points[j].y;
		p.z = pointcloudB->points[j].z;
		p.r = 0;
		p.g = 255;
		p.b = 0;
		MergePointCloud->points.push_back(p);
	}
	for (int j = 0; j < pointcloudA_SVD->points.size(); j += 1) {
		pcl::PointXYZRGB p;
		p.x = pointcloudA_SVD->points[j].x;
		p.y = pointcloudA_SVD->points[j].y;
		p.z = pointcloudA_SVD->points[j].z;
		p.r = 0;
		p.g = 0;
		p.b = 255;
		MergePointCloud->points.push_back(p);
	}
	for (int j = 0; j < pointcloudA_LM->points.size(); j += 1) {
		pcl::PointXYZRGB p;
		p.x = pointcloudA_LM->points[j].x;
		p.y = pointcloudA_LM->points[j].y;
		p.z = pointcloudA_LM->points[j].z;
		p.r = 255;
		p.g = 255;
		p.b = 0;
		MergePointCloud->points.push_back(p);
	}
	for (int j = 0; j < pointcloudA_DQ->points.size(); j += 1) {
		pcl::PointXYZRGB p;
		p.x = pointcloudA_DQ->points[j].x;
		p.y = pointcloudA_DQ->points[j].y;
		p.z = pointcloudA_DQ->points[j].z;
		p.r = 255;
		p.g = 0;
		p.b = 255;
		MergePointCloud->points.push_back(p);
	}
	MergePointCloud->height = 1;
	MergePointCloud->width = MergePointCloud->points.size();
	MergePointCloud->is_dense = false;
	pcl::io::savePCDFile("MergePointCloud.pcd", *MergePointCloud);
	MergePointCloud->points.clear();
	
	PointCloud::Ptr MergeMap(new PointCloud);
	for (int j = 0; j < mapA->points.size(); j++) {
		pcl::PointXYZRGB p;
		p.x = mapA->points[j].x;
		p.y = mapA->points[j].y;
		p.z = mapA->points[j].z;
		p.r = 255;
		p.g = 0;
		p.b = 0;
		MergeMap->points.push_back(p);
	}
	for (int j = 0; j < mapB->points.size(); j++) {
		pcl::PointXYZRGB q;
		q.x = mapB->points[j].x;
		q.y = mapB->points[j].y;
		q.z = mapB->points[j].z;
		q.r = 0;
		q.g = 255;
		q.b = 0;
		MergeMap->points.push_back(q);
	}
	for (int j = 0; j < mapA_SVD->points.size(); j++) {
		pcl::PointXYZRGB q;
		q.x = mapA_SVD->points[j].x;
		q.y = mapA_SVD->points[j].y;
		q.z = mapA_SVD->points[j].z;
		q.r = 0;
		q.g = 0;
		q.b = 255;
		MergeMap->points.push_back(q);
	}
	for (int j = 0; j < mapA_LM->points.size(); j++) {
		pcl::PointXYZRGB q;
		q.x = mapA_LM->points[j].x;
		q.y = mapA_LM->points[j].y;
		q.z = mapA_LM->points[j].z;
		q.r = 255;
		q.g = 255;
		q.b = 0;
		MergeMap->points.push_back(q);
	}
	for (int j = 0; j < mapA_DQ->points.size(); j++) {
		pcl::PointXYZRGB q;
		q.x = mapA_DQ->points[j].x;
		q.y = mapA_DQ->points[j].y;
		q.z = mapA_DQ->points[j].z;
		q.r = 255;
		q.g = 0;
		q.b = 255;
		MergeMap->points.push_back(q);
	}
	MergeMap->height = 1;
	MergeMap->width = MergeMap->points.size();
	MergeMap->is_dense = false;
	pcl::io::savePCDFile("MergeMap.pcd", *MergeMap);
	MergeMap->points.clear();
	std::cout << "**********************************************************" << std::endl;
	std::cout << "green: target" << std::endl;
	std::cout << "red: source" << std::endl;
	std::cout << "blue: SVD estimateRigidTransformation" << std::endl;
	std::cout << "yellow: LM estimateRigidTransformation" << std::endl;
	std::cout << "pink: DQ estimateRigidTransformation" << std::endl;
	std::cout << "**********************************************************" << std::endl<< std::endl<< std::endl;
	return 0;
}