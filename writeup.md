# Project: Perception Pick and Place

---

[//]: # (Image References)

[image01]: ./misc_images/image01.png
[image02]: ./misc_images/image02.png
[image03]: ./misc_images/image03.png
[image04]: ./misc_images/image04.png
[image05]: ./misc_images/image05.png
[image06]: ./misc_images/image06.png
[image07]: ./misc_images/image07.png
[image08]: ./misc_images/image08.png
[image09]: ./misc_images/image09.png
[image10]: ./misc_images/image10.png
[image11]: ./misc_images/image11.png
[image12]: ./misc_images/image12.png
[image_cover2]: ./misc_images/image_cover2.png
[image_cover1]: ./misc_images/image_cover1.png



## Introduction

The objective of this project was to construct a perception pipeline to allow the PR2-robot to recognize specific objects in a cluttered environment for pick and place operations. The pipeline takes as input noisy data from the robot's RGB-D camera and outputs .yaml files containing objects labels, their pick and place positions, and the arm to be used during these operations.

The image below shows an example of one of the cluttered environments (in Gazebo) where the perception pipeline will be tested:

![alt text][image_cover2]

Here, the same environment in Rviz:

![alt text][image_cover1]


## Perception pipeline construction

The pipeline that was constructed to handle the camera data is contained in the *plc_callback()* function of the *project_template.py*. Each of the steps that were taken are described in the sections below.


### Part 1: Filtering and RANSAC plane fitting
In this part of the perception pipeline filtering techniques were used to clean the data and isolate the objects of interest in the scene below:

![alt text][image01]


#### Statistical Outlier Filtering
After converting the camera data to a point cloud, a statistical outlier filter was used to remove noise caused by external factors (dust and humidity in the air, light sources, etc.). This filter takes each point in the cloud, computes the mean distance to its neighbors, and discards points that are outside a global interval (mean + standard deviation). 

In this case, a *mean of 10* and *standard deviation of 0.5* were used. The image below shows the reduction of outliers (noise) thanks to this filtering technique:

![alt text][image02]
 

#### Voxel Grid Downsampling
Next, the data was downsampled in order to keep enough information for succesful object recognition and a lower computation time than that of dense point clouds.
To do this, a voxel grid filter was used. After dividing the point cloud into a grid of voxels of a specific size, the filter takes the average of the points within each voxel, resulting in a less populated point cloud. 

A *leaf size (or voxel size) of 0.003* was used, because it retained a sufficient amount of information to recognize the objects in the upcoming test scenes. I also tried values from 0.01 to 0.005, but the chosen value lead to the best performance.

![alt text][image03]


#### Pass Through Filters
Since we have prior knowledge of the gross location of the objects of interest in the scene, a series of pass through filters were used to extract this region of interest and discard useless data.

First, a filter was applied in the Z-axis direction (*axis_min = 0.606 to axis_max = 1.1*) such that only the table and objects were preserved in the point cloud. Next, another filter was applied in the Y-axis direction (*axis_min = -0.55 to axis_max = 0.55*) to remove the dropboxes' edges:

![alt text][image04]


#### RANSAC Plane Segmentation
Now, our point cloud contains the objects and the table. However, we are specifically interested in the objects only. So, how to remove the table from the point cloud? The answer is: using RANSAC plane segmentation.

This technique can be used to identify points belonging to a specific model. In our case, all points belonging to the table can be described by a plane model. Using this model, the algorithm divided the points into to  2 groups: inliers (table points) and outliers (object points):


![alt text][image05]

![alt text][image06]

### Part 2: Clustering for segmentation
Having obtained a point cloud containing only the objects of interest, the next step consists in segementing it into individual objects or "clusters".

For this purpose euclidean clustering was used. This algorithm segments the data into groups based on the distance between points. Therefore, points packed closer together are classified as belonging to the same cluster or object.

This algorithm was used with a *distance threshold of 0.05*, a *min. cluster size of 30 points*, and a *max. cluster size of 7000 points*. Initially, a max cluster size of 2500 seemed to work for various objects, but when larger objects were introduced in the test scene 3 (e.g. snack box) the cluster size had to be incremented.

The image below shows how the objects were segmented into their own cluster, shown by the different colors:

![alt text][image07]


### Part 3: Feature extraction, SVM training and Object recognition

The next step was to extract useful features (or characteristics) that would allow the robot to recognize (or assign a label) to the previous objects.

#### Feature extraction
These features were: color histograms, which capture color information, and normal histograms, which capture shape information. For the color histograms, the *HSV color space* was used because, as opposed to the RGB space, it is not sensitive to light conditions, and thus, lead to a more robust object recognition in all test scenes.


#### SVM training 
Next, a SVM (support vector machine) was trained for object recognition using these features.
The training set was generated by spawning the objects (of all 3 upcoming test scenes) in *100 random orientations*, extracting their features, and assigning them to their known labels.

The number of orientations was initially 5, then 15, and gradually increased to 100, when the SVM succeeded in correctly classifying almost all objects in the test scenes.

The training resulted in an SVM model with a classification *accuracy of 97% (+/- 0.01)*, as shown by the following confusion matrix:

![alt text][image12]

Note: the functions used for capturing the features (capture_features.py) and training the SVM (train_svm.py) can be found in [this](https://github.com/Tonks89/RoboND-Perception-Training) repository (which is the sensor_stick repo in the lessons).



#### Object recognition
Finally, the trained SVM model was used to recognize each of the objects in our initial scene (now separated into different clusters). These objects (or clusters) were assigned their own label and added to a list of detected objects.


![alt text][image08]





## Pick and Place Setup
Lastly, a function (*pr2_mover()*) was designed to read in a list of objects for the robot to pick and construct a message containing the necessary information to perform pick and place operations.

* This function takes in the list of detected objects in the scene and compares each of them to the objects in the pick list. 

* If an object in the pick list was detected, then the following information is determined:

1) Test scene number (*test_scene_num*): simply, the number of the scene currently running, which contains the object.

2) Pick position (*pick_pose*): determined by calculating the centroid from the object's point cloud (the cloud given by the perception pipeline).

3) Place position (*place_pose*): determined by retrieving the center-bottom coordinates of the appropriate dropbox (the dropbox name is given in the pick list alongside each object).

4) Arm used for picking (*arm_name*): determined by checking in which dropbox the object should be placed (green box - right arm, red box - left arm).

* For each object, this information is stored as a dictionary. 

* Once each item in the pick list has been checked for detection, all the data is converted into a .yaml file.

* Finally, the above information can be used as a message for performing pick and place operations.


## Results

Thanks to the perception pipeline the robot was able to correctly recognize the objects in the scene.

In scene 1 he recognized 3/3 objects:


![alt text][image09]


In scene 2 he recognized 4/5 objects:

![alt text][image10]


In scene 3 he recognized 8/8 objects:

![alt text][image11]

Finally, thanks to the pick and place setup, .yaml files were succesfully created for each scene. These files contain the relevant information for each of the objects detected from the pick list (all .yaml are stored in the 'Output' folder in this repository).

However, although the perception pipeline works really work with all 3 scenes, I would like to tune it better in order to detect all objects successfully. Specifically, because in scene 2, the object 'book' is always mistaken for an object 'soap'. To improve this, perhaps the SVM should be trained with a wider variety of 'book' poses.

Furthermore, in the future, I would like to work on going beyond the project requirements and finish the pick and place operations.



