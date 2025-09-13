Have you ever wanted to know the trajectory of a robot with respect to the map based solely on its odometric readings but you also at the same time need to use a localisation method? Is your initial pose being kind-of off so you first need to move the robot so that its pose estimate has converged but doing so dislocates the `base_footprint` frame from `odom`, thereby destroying the desired odometry?

Worry no more: this package does exactly what you need. You `bringup` your robot, and `base_footprint` coincides with `odom`. Your initial pose estimate is off and therefore you move the robot slightly to the left and to the right and forward and backward, and in the process, since the localisation method that you use all the time accounts for odom drift, and motion has happened, the two frames no longer coincide. Once initialisation has happened, you now want to know `odom` from here on out with respect to `map` (or the robot's initial reliable pose as a reference frame). You issue

```bash
roslaunch odom_logger avanti_odom_logger.launch
```

and then set the new odom frame to where the robot is by issuing

```bash
rosservice call /odom_logger/set_initial_pose
```

When you are indeed ready to log odometry you issue

```bash
rosservice call /odom_logger/start
```
