[//]: # (Image References)
[image_1]: ./images/voxel_downsampled.pcd
[image_2]: ./images/pass_through_filtered.pcd
[image_3]: ./images/extracted_outliers.pcd
[image_4]: ./images/extracted_inliers.pcd
[image_5]: ./images/pcl_table.png
[image_6]: ./images/pcl_objects.png
[image_7]: ./images/pcl_cluster.png
[image_8]: ./images/figure_1_CMWN.png
[image_9]: ./images/figure_2_NCM.png
[image_10]: ./images/test_1.png
[image_11]: ./images/test_2_1.png
[image_12]: ./images/test_world_3.png


# 3dPerception
***Perception and Object Recognition. In this project we will use perception , filtering , clustering, segmentation, feature extraction, SVM training and object recognition techniques to identify objects on a table using PR2 robot in a simulated environment. The PR2 robot is outfitted with RGB-D Camera to perceive its surrounding and using techniques mentioned and used PR2 robot will be able to identify the target objects from a so-called "Pick List" in a  particular order. *** 

The setup environment is used from the below repository

https://github.com/udacity/RoboND-Perception-Project

## Exercise 1, 2 and 3 Pipeline Implemented

### Pipeline for filtering and RanSac Plane Segmentation implemented (Exercise-1)

In brief, the steps to complete this exercise are the following:

Downsample the point cloud by applying a Voxel Grid Filter.

Apply a Pass Through Filter to isolate the table and objects.

Perform RANSAC plane fitting to identify the table.

Use the ExtractIndices Filter to create new point clouds containing the table and objects separately.


### The results after all the steps are as follows (Download Exercise-1 folder in https://github.com/udacity/RoboND-Perception-Exercises repository and copy the Ransac.py file from here )

***Run - python Ransac.py and the following files are created***

You can view these files using pcl_viewer command.

Voxel Grid Filter
![Voxel Grid Filtered][image_1]

Pass Through Filter
![Pass Through Filter][image_2]

Extract Indices for table
![Table from extract indices][image_3]

Extract Indices for objects
![Objects from extract indices][image_4]

### Clustering and Segmentation implemented (Exercise-2)

Create publishers and topics to publish the segmented table and tabletop objects as separate point clouds

Apply Euclidean clustering on the table-top objects (after table segmentation is successful)

Create a XYZRGB point cloud such that each cluster obtained from the previous step has its own unique color.

Finally publish colored cluster cloud on a separate topic

Added segmentation.py file in sensor_stick/scripts folder.

***The above can be checked by using segmentation.py in sensor_stick/scripts folder. Check out the Exercise-2 folder in https://github.com/udacity/RoboND-Perception-Exercises repository for setup.

***Use the following commands - roslaunch sensor_stick robot_spawn.launch and on other terminal use ./segmentaion.py under scripts folder.***

### The published topics are available in Rviz

Table as Point Cloud
![Table][image_5]

Objects as Point Cloud
![Table][image_6]

Cluster as Point Cloud
![Table][image_7]

### Features extracted and SVM trained. Object recognition implemented.(Exercise-3)

Extract colors and shape features from the objects segmented above.Extract a training set of features and labels then train an SVM to recognise specific object in the scene and then use classifier to predict what objects are present in the sgemented point cloud.

Added compute_color_histograms() and compute_normal_histograms() function in features.py located in Exercise-3/sensor_stick/src/sensor_stick in https://github.com/udacity/RoboND-Perception-Exercises repository.

Modified capture_features.py for specific models. Modified train.svm for resolving error for Cross_validation module.

Use the following commands to extract features and then train the SVM to get Normalised Confusion Matrix . Setup everything as mentioned in https://github.com/udacity/RoboND-Perception-Exercises repository for Exercise-3. Modified capture.py and added obj_recognition.py for detecting labels and showing the labels in sensor_stick/scripts folder.

  ***roslaunch sensor_stick training.launch***
  
  ***rosrun sensor_stick capture_features.py***
  
 After the features are extracted it will create training_set.sav file. Run the following command to train the SVM and generate Normalised Confusion Matrix
 
 ***rosrun sensor_stick train_svm.py***
 
 This would create model.sav file and also create two files as shown below:
 
 Normalised Confusion Matrix
![NCM][image_8]
 
 Confusion Matrix without Normalisation.
![CMWN][image_9]


## For three tabletop setups object recognition perofrmed , pick and place request message constructed to create three outputs respectively.

Follow the setup from https://github.com/udacity/RoboND-Perception-Project repository.

For object recognition and pick and place message construction follow the following steps.

1.Copy sensor_stick from Exercise-3 folder with all the changes/add done as mentioned abovefor Exercise-3 to ~/catkin_ws/src folder parallel to RoboND-Perception-Project folder.

2.Copy model.sav file in RoboND-Pereception-Project folder , from where you would run the command rosrun.

3.Add my_perception.py to pr2_robot/scripts folder.

4.Changed pick_list_1.yaml,pick_list_2.yamland pick_list_3.yaml for adding the test_scene_num parameter. 

4.Make sure that Tabletop 1 is setup in pick_place_project.launch file located in pr2_robot/launch folder. Use test1.world and pick_list_1.yaml at the mentioned place.

Run the following commands on seperate terminals

1.roslaunch pr2_robot pick_place_project.launch.

2.rosrun pr2_robot my_perception.py

Check Rviz for cluster/table/objects/labels

Check for output_1.yaml created.

Repeat the same for test2.world , pick_list_1.yaml and test3.world and pick_list_3.yaml in pick_place_project.launch

Check for output_1.yaml, output_2.yaml and output_3.yaml 

### The following results are acheived for three tabletop setups - 100% object recognised in all of the setups (:-D)

TableTop 1 -test1.world and pick_list_1.yaml
![tabletop 1][image_10]

TableTop 2 -test2.world and pick_list_2.yaml
![tabletop 1][image_11]

TableTop 1 -test3.world and pick_list_3.yaml
![tabletop 1][image_12]







