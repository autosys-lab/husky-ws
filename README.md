husky
=====

:warning: **For ROS 2 Humble, this repository is no longer used.  Please visit [clearpath_common](https://github.com/clearpathrobotics/clearpath_common).  Check the [documentation](https://docs.clearpathrobotics.com/docs/ros/) for more details including supported sensors.**

Common ROS packages for the Clearpath Husky, useable for both simulation and
real robot operation.

 - clearpath_control : Control configuration
 - clearpath_platform_description : Robot description (URDF)
 - husky_msgs : Message definitions
 - husky_navigation : Navigation configurations and demos

For Husky instructions and tutorials, please see [Robots/Husky](http://wiki.ros.org/Robots/Husky).

To create a custom Husky description or simulation, please fork [husky_customization](https://github.com/husky/husky_customization).

husky_desktop
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
