[//]: # (Image References)
[image_1]: ./images/voxel_downsampled.pcd
[image_2]: ./images/pass_through_filter.pcd
[image_3]: ./images/extracted_outliers.pcd
[image_4]: ./images/extracted_inliers.pcd

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

![Pass Through Filter][image_2]

Extract Indices for table
![Table from Extract Indices][image_3]

Extract Indices for objects
![Objects from extrect indices][image_4]



