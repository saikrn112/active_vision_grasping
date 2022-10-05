# active_vision_grasping
To Grasp

## STEPS:
  * Create environment
    * Spawn table, bot/cam, object
    * Create view sphere
  * Get image feed
  * Preprocess ( optional)
    * Subsample
    * filter
  * Stitch PCL (if needed)
  * Finding Normals
  * Approximation( find the normalvectors)
  * All the normals should he pointing towards the centre
  * How to search for best graph? (1. for every single point on point cloud, calculate the angles and check if it is within your threshold)
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
  


* setting up environment - all
* point cloud library learning - all
* filtering - (optional needed only in real world scenarios)
* downsampling - (optional needed only in real world scenarios)
* stitching the point cloud - do we really need it?  (bonus)
* segementation  - (15%)
* get normals -
    * finding the angles
    * normals could be in either direction since PCL doesnt know 
        the object
    * give reference point normals to point
        * find the reference point
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
