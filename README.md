# ros_custom_controller
Repository of my custom controller created in ROS. By using the `ros_control` package, I implemented a Computed Torque Control approach to control a KUKA lwr arm robot.

## Example

## Installation/Build

1) Required dependences:
- [orocos_kdl](http://wiki.ros.org/orocos_kdl)
- [kdl_parser](http://wiki.ros.org/kdl_parser)
- [controller_manager](http://wiki.ros.org/controller_manager)
- [hardware_interface](http://wiki.ros.org/hardware_interface)
- [controller_interface](http://wiki.ros.org/controller_interface)
- [pluginlib](http://wiki.ros.org/pluginlib)
- [std_msgs](http://wiki.ros.org/std_msgs)
- [sensor_msgs](http://wiki.ros.org/sensor_msgs)

2) Clone the repository into your `catkin workspace` and build all the packages.

## Running

Run the controller and hardware interface using `roslaunch`:
```sh
$ cd [your catkin_workspace]/src/arm_controller 
$ roslaunch arm_effort_controller.launch 
```

Then run the the robotic arm simulator node:  
```sh
$ rosrun robot_kdl robot_sim_node _internal_control:=false
```

