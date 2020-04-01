# Planning-for-Autonomous-Robots

ENPM6613Planning for Autonomous RobotsPlanning is a fundamental capability needed to realize autonomous robots. Planning in the context of autonomous robots is carried out at multiple different levels. At the top level, task planning is performed to identify and sequence the tasks needed to meet the mission requirements. At the next level, planning is performed to determine a sequence of motion goals that satisfy individual task goals and constraints. Finally, at the lowest level, trajectory planning is performed to determine actuator actions to realize the motion goals. Different algorithms are used to achieve planning at different levels. This graduate course will introduce planning techniques for realizing autonomous robots. In addition to covering traditional motion planning techniques, this course will emphasize the role of physics in the planning process. This course will also discuss how the planning component is integrated with control component. Mobile robots will be used as examples to illustrate the concepts during this course. However, techniques introduced in the course will be equally applicable to robot manipulators RoboticsCore

## 1.Chance Constrained RRT [Github Link](https://github.com/ChoLiu/Planning-for-Autonomous-Robots/tree/master/Chance%20Constrained%20RRT)

Please check the folder for detail of this project

### RRT

<img src= "Chance Constrained RRT/RRT.gif" width="1000">

- The blue branches represent the randomly generated node paths
- The black shapes are our obstacles
- Ted outline is the applied minkowski
- The green circles represent the uncertainty and the path
As you can see, the further the distance traveled, the uncertainty grows. For just regular RRT algorithm, it chooses this risky path in between the two rectangular obstacles. It does not check whether the area of uncertainty will collide with an obstacle.

### CC-RRT

<img src= "Chance Constrained RRT/CC-RRT.gif" width="1000"> 

CC-RRT we assume that as the distance traveled by the robot  increases, the robot’s uncertainty  also increases. The algorithm includes an additional condition that checks the probability of robot uncertatinty is less than the probability of collision which is based on the probability of safety that is set by the user. As the result, for the same map CC-RRT obtained a path with lower collision chance.

### Car Lane Change Application

<img src= "Chance Constrained RRT/Lane Change.gif" width="1000">

- The green rectangle represents the car need to pass
- The yellow rectagles represent the car movements while passing the target car

## 2.A* and Dijkstra [Github Link](https://github.com/ChoLiu/Planning-for-Autonomous-Robots/tree/master/A*%20and%20Dijkstra)

### A*
<img src= "A* and Dijkstra/Astar.gif" width="1000"> 

### Dijkstra
<img src= "A* and Dijkstra/Dijkstra.gif" width="1000"> 

