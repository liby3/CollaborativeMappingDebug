# CollaborativeMappingDebug

Collaborative Mapping Debug, test Matching

![Screenshot](/Connect.png)
# Environment:

Your computer should under the following environments:
-   ubuntu16.04 + ROS Kinetic

-   pcl1.7

-   cmake

-   eigen3


# How to build

```
$ cd ~/CollaborativeMappingDebug
$ cmake .
$ make 
```

# Running:

```
./main

```

# The result:
Two global maps frame mapping result, please check MergePointCloud.pcd
Two global whole maps result, please check MergeMap.pcd

```
# view some pcd file
pcl_viewer xx.pcd

# connect all the pcd files and generate a new one whose name is output.pcd
pcl_concatenate_points_pcd *.pcd
```
# Other instructions
folder /PCD2 and /PCD3 contains global map: 

-	Surf-1.pcd is the first frame global map in CarB;

-	Surf-223.pcd is the last frame global map in CarA;

-	output.pcd in each folder is the whole global map in CarA and CarB.