# active_vision_grasping
To Grasp

## STEPS:
* setting up environment - all
* point cloud library learning - all
* stitching the point cloud - do we really need it?  (bonus)
* segementation  - Janie & Hasitha
* get normals - Ramana & Tript
* spawn objects - Laddu & Saurabh
	* sphere on a table
	* cuboid on a table
	* glass vase on a table
* grasp calculation - 
	* find optimal grasp
	* moving the camera
	* probably do it in another node

* move camera optimally (optional)
* move robot to grasp (optional)
* filtering - (optional needed only in real world scenarios)
* downsampling - (optional needed only in real world scenarios)


## Steps to run environment
1. ```cd $workspace_dir$```
2. ```colcon build --packages-selct vbm_project_env```
3. ```ros2 launch vbm_project_env simulation_can.launch.py```