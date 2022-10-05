# active_vision_grasping
To Grasp

## STEPS:
  * Create environment
    * Spawn table, bot/cam, object
    * Create view sphere
  * Get image feed
  * Preprocess
    * Subsample
    * filter
  * Stitch PCL (if needed)
  * Find grasp points
  * Move the cam
  1. Create environment
    - Spawn table, bot/cam, object
    - Create view sphere
  2. Get image feed
  3. Preprocess
    - Subsample
    - filter
  4. Find grasp points
  5. 


* setting up environment - all
* point cloud library learning - all
* filtering -
* downsampling -
* stitching the point cloud - do we really need it?  (bonus)
* segementation  -
* get normals -
* spawn objects -
	* sphere on a table
	* cuboid on a table
	* glass vase on a table
* grasp calculation -
	find optimal grasp
	moving the camera
	how do we assign tangential axis once we get the normal vectors
	probably do it in another node

* move camera optimally (optional)
* move robot to grasp (optional)
