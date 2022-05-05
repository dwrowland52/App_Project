# App_Project

* Has three directories:
  * Scripts:
    * find_pt_cloud_Diff.py - Holds created function to find and return defect locations for PCD files
    * compare_pt_clouds.py - Non-function version of find_pt_cloud_Diff.py that will visualize .pcd files and manipulate them to isolate defects
    * Transform_Script.py - Takes .pcd files / numpy arrays of XYZ coordinates in the optical reference frame and transforms them into the base link reference frame
    * EE_pose_Publisher.py - ROS publisher node used to get EE XYZ coordinates using MoveIt and publish it to created ROS topic
    * Near_Defect.py - Creates a ROS node that listens to EE_XYZ topic and publishes twist commands when EE is near a defect location
  * Figures:
    * Contains screen shots used in the presentation slides
    * The video demo is within our presentation slides, we tried to add it but the file size was too large
  * PCD:
    * Holds the three .pcd files
    * point_test_data2_no_defects_###.pcd for defect free work piece
    * point_test_data2_###.pcd for work piece with defects
    * def_location.pcd containing isolated defects

## Steps to Download:

* To run only the python scripts, clone this repo:
```
git clone https://github.com/dwrowland52/App_Project.git
```
* To download the full demo (Meredith):
```
git clone https://github.com/dwrowland52/App_Project.git
mkdir -p ~/demo_ws/src
cd ~/demo_ws/src
git clone https://github.com/UTNuclearRobotics/tvar_planner_sim_demo.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
sudo apt install ros-noetic-rviz-animated-view-controller
pip install open3d
pip install pytransform3d
```
* Add the files in App_Project/PCD to ~/demo_ws. To run the demo, these files need to be in you pwd. The other files in App_Project are unnecessary, since theyre included in tvar_planner_sim_demo.git

## Steps to Run the Demo:
* Make sure demo_ws is sourced in all terminals:
```
source devel/setup.bash
```
or add this to your .bashrc:
```
source ~/demo_ws/devel/setup.bash
```
* Launch gazebo and tvar_planner. Allow a few seconds between each of these commands. 
```
roslaunch ur_gazebo ur3_bringup.launch gui:=false
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true
roslaunch tvar_planner ur3_simulation_imarkers_wip.launch tvf_id:=104 sim:=false
```
* Launch python nodes and echo the delta_jog_cmds topic if you want to know when the end effector is near a defect:
```
rosrun moveit_tutorials EE_pose_Publisher.py
rosrun moveit_tutorials Near_Defect.py 
rostopic echo /jog_arm_server/delta_jog_cmds 
rosrun pcl_ros pcd_to_pointcloud defect_cloud_base_link.pcd 1
```
* In Rviz, move the interactive TVF so the defects displayed in the point cloud are on top of the workpiece. Rotate the workpiece very slightly about one of the horizontal axes. Right click any of the green waypoints and select "Move to Pose". If the end effector approaches a defect, you will see twist messages being published to /jog_arm_server/delta_jog_cmds.


### *To Visualize Pointclouds and Transformations (For Students):*
* Pip install Open3d and pytransform3d
```
pip install open3d
pip install pytransform3d
```
* Run scripts compare_pt_clouds.py or Transform_Script.py in desired IDE
