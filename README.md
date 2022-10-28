### Lab 5

The code in ariac_node.cpp takes in order information and camera sensor data by subscribing to those topics. From that information, the location of the products within the order are then printed out. The relative location of the products is found by doing a transformation of the camera location and relative part location with respect to the cameras is done.

```bash
#RUN code from roslaunch
roslaunch ariac_group2 run_comp.launch &
```
