[//]: # (Image References)
[image_1]: ./images/voxel_downsampled.pcd
[image_2]: ./images/pass_through_filtered.pcd
[image_3]: ./images/extracted_outliers.pcd
[image_4]: ./images/extracted_inliers.pcd
[image_5]: ./images/pcl_table.png
[image_6]: ./images/pcl_objects.png
[image_7]: ./images/pcl_cluster.png

# 3dPerception
Perception and Object Recognition


## Exercise 1, 2 and 3 Pipeline Implemented

### Pipeline for filtering and RanSac Plane Segmentation implemented

In brief, the steps to complete this exercise are the following:

Downsample the point cloud by applying a Voxel Grid Filter.

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

### Clustering and Segmentation implemented 

Create publishers and topics to publish the segmented table and tabletop objects as separate point clouds

Apply Euclidean clustering on the table-top objects (after table segmentation is successful)

Create a XYZRGB point cloud such that each cluster obtained from the previous step has its own unique color.

Finally publish colored cluster cloud on a separate topic

### The published topics are available in Rviz

Table as Point Cloud
![Table][image_5]

Objects as Point Cloud
![Table][image_6]

Cluster as Point Cloud
![Table][image_7]

### Features extracted and SVM trained. Object recognition implemented.






