Husky
=====

This repository contains the ROS packages for the Clearpath Husky robot.
It is based on the [clearpath_common](https://github.com/clearpathrobotics/clearpath_common) repository, which contains the common ROS packages for all Clearpath robots.

For further information on the Husky, please see the [Husky documentation](https://docs.clearpathrobotics.com/docs/ros/).

Common ROS packages for the Clearpath Husky, useable for both simulation and
real robot operation.

 - clearpath_control : Control configuration
 - clearpath_platform_description : Robot description (URDF)
 - husky_msgs : Message definitions
 - husky_navigation : Navigation configurations and demos

clearpath_desktop
=============

Desktop ROS packages for the Clearpath Husky, which may pull in graphical dependencies.

 - clearpath_viz : Visualization (rviz) configuration and bringup

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky

husky_robot
===========

Robot ROS packages for the Clearpath Husky, for operating robot hardware.

 - husky_bringup : Bringup launch files and scripts.
 - clearpath_common : Hardware driver for communicating with the onboard MCU.

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky

husky_simulator
==============

Simulator ROS packages for the Clearpath Husky.

 - husky_gazebo : Gazebo plugin definitions and extensions to the robot URDF.

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky
