[//]: # (Image References)
[image_1]: ./images/Voxel.pcd
[image_2]: ./images/Passthrough.pcd
[image_3]: ./images/ransac.pcd
[image_4]: ./images/tabletop.pcd
[image_5]: ./images/object.pcd

# 3dPerception
Perception and Object Recognition


## Exercise 1, 2 and 3 Pipeline Implemented

### Pipeline for filtering and RanSac Plane Segmentation implemented

In brief, the steps to complete this exercise are the following:

Downsample your point cloud by applying a Voxel Grid Filter.
Apply a Pass Through Filter to isolate the table and objects.
Perform RANSAC plane fitting to identify the table.
Use the ExtractIndices Filter to create new point clouds containing the table and objects separately.


### The results after all the steps are as follows:

Voxel Grid Filter
![Voxel Grid Filtered][image_1]

Pass Through Filter


RanSaC Plane fitting to identify the table


Extract Indices for table


Extract Indices for objects




