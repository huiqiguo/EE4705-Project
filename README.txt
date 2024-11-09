## EE4705 PROJECT 2 ##
This project implements the STOMP algorithm to carry out collision-avoidance path planning for a robot manipulator in MATLAB.

## TASK 1 ##
To plan a collision-free path for the Kinova Gen 3 robot manipulator and generate an animation, run the KinovaGen3_STOMP_Path_Planning.mlx script.

## TASK 2 ##
To plan a collision-free path for the Kuka Iiwa robot manipulator and generate an animation, run the KukaIiwa_STOMP_Path_Planning.mlx script.

## TASK 3 ##
The updateJointsWorldPosition.m function utilises the helper function FKinSpace.m to calculate the forward kinematics and derive the joint world coordinates using the Product of Exponentials (PoE) formula.

The FKinSpace.m function in turn uses the following helper functions:
1. AxisAng3.m
2. MatrixExp3.m
3. MatrixExp6.m
4. NearZero.m
5. so3ToVec.m
6. VecTose3.m
7. VecToso3.m

## TASK 4 ##
The helperCreateObstaclesKUKAIIWA.m helper function is used to implement obstacles. The obstacles can be seen in the animation that is generated when running the KukaIiwa_STOMP_Path_Planning.mlx script.

## TASK 5 ##
Constraint cost is added when calculating the total trajectory cost using the stompTrajCost.m helper function.

## ADDITIONAL IMPROVEMENTS ##
1. Interpolation of sphere centers is implemented in the stompTrajCost.m helper function to ensure that a consistent number of spheres is used to model the robot manipulator when calculating obstacle cost.