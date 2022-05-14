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
## Draw the map of ICT Classroom
&emsp;&emsp;&emsp; -We hand-animated a 2D map of an ICT classroom for use in an algorithm for path planning.
<img src="./map.jpeg" width=250>
## Choice of Path Planning Algorithms
&emsp;&emsp;&emsp; -At first we were interested in the following three path planning algorithms, which are A*, RRT and RRT* algorithms;
Considering the problem of controlling the drone and the complexity of the path, we finally chose the A* algorithm; because the path given by the A* algorithm is more suitable for the drone.
| A* algorithm                                                                          | RRT algorithm                                                                 | RRT* algorithm                                                                          |
| ------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| [<img src="./a_star.gif" width=250>](lab2/program/PathPlanning/dijkstra.py) | [<img src="./rrt.gif" width=250>](lab2/program/PathPlanning/rrt.py) | [<img src="./rrt_star.gif" width=250>](lab2/program/PathPlanning/rrt_star.py) |

---
## Communication
&emsp;&emsp;&emsp;-At the beginning, the connection between the Socket Server and the Client between the PC and the Jetbot must be established. The Socket Server must always monitor the connection until the drone reaches the end point; (the UAV will send a Flag_1 signal after it reaches the end point); Jetbot accepts After reaching the Flag signal, it will do a circle action.
&emsp;&emsp;&emsp;-Then the drone arrives at the starting point through another path planning. After arriving, the Socket Client will send the Flag_2 signal; Jetbot will stop the circle action after receiving the Flag signal.

![image](https://user-images.githubusercontent.com/71862228/168427106-8f1d4d42-0ae7-43d0-8ebe-5cc1f336e1eb.png)
