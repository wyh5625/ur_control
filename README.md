# Handeye calibration for realsense camera

1. launch the ur5 driver

```
roslaunch example_organization_ur_launch start_ur10e.launch
```

2. launch handeye server program and run realsense camera

```  
roslaunch cobot_control handeye.launch use_realsense:=true
```

3. view the taged image 

```
rosrun image_view image_view image:=/tag_detections_image
```

4. initialize ur5 eef position
```
python auto_handeye_init.py
```

5. run the handeye calibration script
```
python auto_handeye.py
```



# peg-in-hole experiment after camera calibration
(check ur5_bringup.launch whether joint velocity controller is started)
1. launch ur5 driver and camera node
```
roslaunch example_organization_ur_launch ur10e_plus_camera.launch
```

2. publish tf from base to camera
```
roslaunch example_organization_ur_launch base_cam_tf_pub.launch
```

3. run the centroid point converter which transform the centroid point coordinate in camera frame to base fram.
```
rosrun ur_control center_converter.py
```

4. run the centroid tracker
```
rosrun soft_object_tracking soft_object_tracking_node
```

5. run centroid marker publisher, use rqt to publish target centroid and use rviz to show centroid marker and the target marker

```
rosrun soft_object_tracking soft_object_tracking_centroid_marker_singlepoint
rqt (pub /soft_object_tracking/target_centroid)
rviz (change fixed frame to base and add MarkerArray)
```

6. run controller node
```
rosrun soft_object_controller soft_object_controller_node_singlearm
```


# peg-in-hole regrasp experiment
1. launch ur5 driver and camera node
```
roslaunch example_organization_ur_launch ur5_plus_camera.launch
```

2. publish tf from base to camera
```
roslaunch example_organization_ur_launch base_cam_tf_pub.launch
```

3. run the centroid point converter which transform the centroid point coordinate in camera frame to base fram.
```
rosrun ur_control centers_converter.py
```

4. run the camera to world point coordinates transform server
```
python point_transform_server.py
```

5. (Optional)Record bar orientation in the controller program when eef link is calibrated

6. run the tracking algorithm to track multi holes
```
rosrun soft_object_tracking soft_object_tracking_node_realtime_crop_multihole_withholeclass
```

7. run the marker publisher(optional), use rqt to publish (target_centroid, go, zero velocity command, init_data, control_gain and number_of_holes) topics and use rviz to show centroid marker and the target marker

```
rosrun soft_object_tracking soft_object_tracking_centroid_marker_singlepoint
rqt (pub /soft_object_tracking/target_centroid)
rviz (change fixed frame to base and add MarkerArray)
```

8. run controller node
```
rosrun soft_object_controller soft_object_controller_node_singlearm_regrasp
```

# peg-in-hole regrasp experiment with 2 camera
1. launch ur5 driver and camera node
```
roslaunch example_organization_ur_launch ur10e_plus_2camera.launch
(or)
roslaunch example_organization_ur_launch ur10e.launch
roslaunch example_organization_ur_launch two_camera.launch
```

2. publish tf from base to camera
```
roslaunch example_organization_ur_launch base_cam_tf_pub_2camera.launch
```

3. run the hole tracker master\
```
rosrun soft_object_tracking hole_tracker_master 2 6
```

4. run the two camera tracker node
```
rosrun soft_object_tracking hole_tracker 0 6
rosrun soft_object_tracking hole_tracker 1 6
```

5. (Optional)Record bar orientation in the controller program when eef link is calibrated

6. run the tracking algorithm to track multi holes
```
rosrun soft_object_tracking soft_object_tracking_node_realtime_crop_multihole_withholeclass_twocamera
```

7. run the marker publisher(optional), use rqt to publish (target_centroid, go, zero velocity command, init_data, control_gain and number_of_holes) topics and use rviz to show centroid marker and the target marker

```
rosrun soft_object_tracking soft_object_tracking_centroid_marker_singlepoint
rqt (pub /soft_object_tracking/target_centroid)
rviz (change fixed frame to base and add MarkerArray)
```

8. run controller node
```
rosrun soft_object_controller soft_object_controller_node_singlearm_regrasp
```


# Setup
1. Connect the robot arm and the pc through router
a. Robot arm should use static ip (eg. 192.168.0.8)
b. Create a new ip on pc: 192.168.0.7
c. Wire robot and pc to router

2. Calibrate the camere with the robot arm
a. Remove gripper and adhere the Tag on the eef
b. (optional) Preset the joint constraints of robot arm by finding the working range using /joint_state topic
c. Move the tag in front of camera.
d. Calibrate two camera

3. Run the program
a. Run the tape orientation calibration script to find coordinates of its two center, then record the values in the corresponding place of program
b. Find the ready position for robot arm eef, which is for preventing the arm from occluding camera when it is finding the next picking place, and update it in the controller program.
c. Update the constraints of joints in moveit_commander.cpp
d. Modify the number of holes in the controller program (i.e. NON).
e. Run rqt which should already has the required topics, modify number of holes in the rqt console.


# Find the marker(peg) centroids
1. launch the ur10e driver
    roslaunch example_organization_ur_launch start_ur10e.launch
2. publish tf between base_link and camera
    roslaunch example_organization_ur_launch base_cam_tf_pub_2camera.launch
3. launch handeye server program and run realsense camera (use camera=:camera0 param in order to connect the tf tree between tag and the baselink)
    roslaunch cobot_control handeye.launch use_realsense:=true camera:=camera0
4. view the taged image 
    rosrun image_view image_view image:=/tag_detections_image
5. run the marker centroid publishing program
    rosrun ur_control centroid_marker_tracking.py
6. read centroid coordinates from topic
    rostopic echo /marker_centroid

# camera 0
[0.4974135756492615, 0.05328924581408501, 0.13864444196224213, 
0.6348020434379578, -0.12575747072696686, 0.13615866005420685, 
0.7794445753097534, 0.0009344826685264707, 0.14056214690208435, 
0.7445365786552429, 0.18573789298534393, 0.13897360861301422, 
0.5939896702766418, 0.21748550236225128, 0.1408877819776535]

# camera 1
[0.48621830344200134, 0.04506840929389, 0.14286966621875763, 0.6235339641571045, -0.13043244183063507, 0.14137694239616394, 0.7634181976318359, -0.007298346608877182, 0.14541374146938324, 0.7360002994537354, 0.1814725399017334, 0.13637366890907288, 0.5825020670890808, 0.21226249635219574, 0.13854961097240448]



