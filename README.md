# RAS_OpenProject
Open project for RAS course


---

## The System
&emsp;&emsp;&emsp; -Tello Drone,Jetbot and Their front camera <br />
&emsp;&emsp;&emsp; -Personal laptop to run the code through extra wireless card and ZeroTier <br />
&emsp;&emsp;&emsp; -Algorithm (A* Path Planning Algorithm) <br />
&emsp;&emsp;&emsp; -Communication (Python Socket Server and Client) <br />
&emsp;&emsp;&emsp; -DataFlow (shows as below) <br />

![image](https://user-images.githubusercontent.com/71862228/168425580-b627fbd7-3d8f-4bbb-94d8-56436c87244d.png)

## Choice of Path Planning Algorithms
&emsp;&emsp;&emsp; At first we were interested in the following three path planning algorithms, which are A*, RRT and RRT* algorithms;
Considering the problem of controlling the drone and the complexity of the path, we finally chose the A* algorithm; because the path given by the A* algorithm is more suitable for the drone.
| A* algorithm                                                                          | RRT algorithm                                                                 | RRT* algorithm                                                                          |
| ------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| [<img src="./a_star.gif" width=250>](lab2/program/PathPlanning/dijkstra.py) | [<img src="./rrt.gif" width=250>](lab2/program/PathPlanning/rrt.py) | [<img src="./rrt_star.gif" width=250>](lab2/program/PathPlanning/rrt_star.py) |

---
