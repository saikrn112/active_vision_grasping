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
2. ```colcon build --packages-selct segmentation```
3. ```ros2 launch segmentation simulation_can.launch.py```
4. Publishing joint position via topic
```
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.0
```
