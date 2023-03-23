# rtioms_recovery_behaviour
## Introduction

- It's a type of recovery behaviour that robots engage in when they encounter obstacles in the way of their goal. It will help the robot to avoid obstacles and make a new path towards the goal point so that it is able to reach it. 
- The rtioms_recovery_behaviour is a combination of two recovery behaviors,
  - backward_recovery
  - rotate_recovery

- A basic function of this algorithm is that when a robot encounters an obstacle, it moves backwards and rotates 360 degrees to find a new path. 
  Traditionally, recovery behavior only rotates 360 degrees and it won't maintains the pose of the robot. There will be no backward movement.
- In the case that the robot touches the critical zone on the costmap (obstacle), the traditional rotation recovery behavior will not be able to recover it because it cannot perform a 360 degree rotation. In our case, it will create the room by itself by moving a certain distance backwards to perform the 360-degree rotation. It will always maintain pose of the robot.

## Writing the recovery behaviour class

### The necessary parts are:

- Include core ROS libraries

- The class should inherits from the interface “nav_core::RecoveryBehavior”

- “initialize” and “makePlan”.

- “#include<pluginlib\class_list_macros.h>”

- Register this planner as a RecoveryBehavior plugin by adding “PLUGINLIB_EXPORT_CLASS(rtioms_recovery_behaviors::RtiomsRecoveryBehaviors, nav_core::RecoveryBehavior)”.

- Now the recovery planner class is done to compile.

- In terminal “catkin build”.

- Then it will create library file in the “lib” directory “~/catkin_ws/devel/lib/librtioms_recovery_behavior”



## Engaging with the navigation stack

### 1) rtioms_recovery_behavior_plugin.xml

- Write down the library path, class name, description of the file.

<img src="images\Rtioms_Plugin.png" alt="logo" style="zoom:100%; margin-left: auto; margin-right: auto; display: block;" />

### 2) move_base.cpp

- Add the recovery behavior sequence in move_base.

<img src="images\MoveBase.png" alt="logo" style="zoom:100%; margin-left: auto; margin-right: auto; display: block;" />

- Save and compile the workspace.
- Run the turtlebot3 simulation and see the recovery behavior sequence in rqt_multiplot.



## Proposed recovery algorithm

<img src="images\Rtioms_Algorithm.png" alt="logo" style="zoom:100%; margin-left: auto; margin-right: auto; display: block;" />

## Results

### Traditional Navigation Method (Pose of the robot not maintained)

- The conventional navigation method of avoiding obstacles while meeting the obstacle was effective in avoiding the obstacle, but still, it was not able to maintain the position of the robot. After the robot met the obstacles, it continued to move in a backward direction until it reached the goal point by moving continuously in that direction.

<img src="images\TraditionalNavigationBehaviour.png" alt="logo" style="zoom:100%; margin-left: auto; margin-right: auto; display: block;" />

### Proposed navigation Method (Pose of the robot maintained)

- The proposed navigation method of avoiding obstacles while meeting the obstacle was effective in avoiding the obstacle, and also, it was able to maintain the position of the robot. In order to maintain the robot's pose, the robot backed up some distance backwards and rotated by itself after meeting obstacles. Then moved to the goal point.

<img src="images\ProposedNavigationBehaviour.png" alt="logo" style="zoom:100%; margin-left: auto; margin-right: auto; display: block;" />
