# home-service-robot
### Repository Description:
Final project for 'Robotics Software Engineer' of Udacity

### Summary of Packages:
#### ROS Packages
* [slam_gmapping](https://github.com/ros-perception/slam_gmapping)
  * This package contains a ROS wrapper for OpenSlam's Gmapping which provides laser-based SLAM. The gmapping is used to create a 2-D occupancy grid map of the environment from laser and pose data collected by a mobile robot.
* [turtlebot](https://github.com/turtlebot/turtlebot)
  * This package provides all the basic drivers for running and using a TurtleBot. The turtlebot_teleop (keyboard_teleop.launch) is used to manually control the robot when creating a map using SLAM.
* [turtlebot_interactions](https://github.com/turtlebot/turtlebot_interactions)
  * This package is used to launch RViz which visualizes what the robot is sensing, planning and acting, and allows us to debug a robot application from sensor inputs to planned actions.
* [turtlebot_simulator](https://github.com/turtlebot/turtlebot_simulator)
  * This package provides turtlebot_gazebo which allows us to launch the turtlebot world and launch the AMCL algorithm which enables a robot localize itself probabilistically using a particle filter.

#### My Own Packages
* [pick_objects](https://github.com/gyanr0425/home-service-robot/tree/main/src/pick_objects)
  * This package commands a robot with goals of pick-up and drop-off zone. The robot go to the pick-up zone, get a virtual object, move to the drop-off zone, and drop the object.
* [add_markers](https://github.com/prasunnyD/Home-Service-Robot/tree/master/src/add_markers)
  * This package publishes a marker of a virtual object in Rviz, which is at the pick-up zone initially. According to the robot actions, the object disappears when the robot reaches the pick-up zone, and appears again when the robot reaches the drop-off zone.
