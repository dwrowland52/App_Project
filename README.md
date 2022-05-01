# App_Project

* Has three directories:
  * Scripts:
    * find_pt_cloud_Diff.py - Holds created function to find and return defect locations for PCD files
    * Transform_Script.py - Takes .pcd files / numpy arrays of XYZ coordinates in the optical reference frame and transforms them into the base link reference frame
    * EE_pose_Publisher.py - ROS publisher node used to get EE XYZ coordinates using MoveIt and publish it to created ROS topic
    * Near_Defect.py - Creates a ROS node that listens to EE_XYZ topic and publishes twist commands when EE is near a defect location
  * Figures:
    * Contains screen shots used in the presentation slides
  * PCD:
    * Holds the three .pcd files
    * point_test_data2_no_defects_###.pcd for defect free work piece
    * point_test_data2_###.pcd for work piece with defects
    * def_location.pcd containing isolated defects

# Steps to run:
 1) Use Ubuntu 20.04
 2) Install Python packages:
    * ROS
    * Open3d
    * Pytransform3d
    * Numpy
 3) Download this repo
 4) Rosrun the two node scripts
 5) Enjoy
