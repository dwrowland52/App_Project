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
  * PCD:
    * Holds the three .pcd files
    * point_test_data2_no_defects_###.pcd for defect free work piece
    * point_test_data2_###.pcd for work piece with defects
    * def_location.pcd containing isolated defects

## Steps to run:

* To run only the python scripts, clone this repo:
'''
git clone https://github.com/dwrowland52/App_Project.git
'''
* To run the full demo:
'''
git clone https://github.com/UTNuclearRobotics/tvar_planner_sim_demo.git
'''

### *For ROS Nodes:*
* Have ROS Noetic installed
* Install MoveIt - Binary install
* Follow along with MoveIt tutorial
* Their demo uses a panda manipulator we will be adding the UR3 to the work space
* Download the config file in your source directory with: git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
* Add our node .py scripts to: /home/’UserName’/ws_moveit/devel/lib/moveit_tutorials/doc/move_group_python_interface/scripts
* In Near_Defect.py - update file paths for clean and defect PCD files
* Make the script executable and add it to the Cmakelists.txt
* Catkin Build and source the workspace:
  * source ~/ws_moveit/devel/setup.bash
* Ready to go

### *For Data Cleaning:*
* Pip install Open3d and pytransform3d
* Run scripts compare_pt_clouds.py or Transform_Script.py in desired IDE
