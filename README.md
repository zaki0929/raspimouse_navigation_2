# raspimouse_navigation_2
Package for implement navigation with IMU, expansion resettings, and original recovery behaviors on Raspberry Pi Mouse


# Requirements

This package requires the following to run:

* Ubuntu
  * Ubuntu 16.04 (Ubuntu 16.04 Server recomended)
* ROS
  * Kinetic Kame
* ROS Package
  * Customized ROS Navigation Stack - [zaki0929/navigation](https://github.com/zaki0929/navigation)
  * IMU Driver - [AtsushiSaito/lab_usb_9axisimu_driver](https://github.com/AtsushiSaito/lab_usb_9axisimu_driver)

# Installation

First of all, install the latest stable version of ROS.
Please refer to [ROS WiKi](http://wiki.ros.org/kinetic/Installation) for installation.

Next, download the dependent ROS package into `~/catkin_ws/src` and build it.

```
cd ~/catkin_ws/src
git clone https://github.com/zaki0929/navigation.git
git clone https://github.com/AtsushiSaito/lab_usb_9axisimu_driver.git
cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash
```

Finally, download this repository and build it.

```
cd ~/catkin_ws/src
git clone https://github.com/zaki0929/raspimouse_navigation_2.git
cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash
```

# Usage

In order to use this package, you need to launch the launch file on Raspberry Pi Mouse side and PC side as follows.

## Raspberry Pi Mouse side

```
roslaunch raspimouse_navigation_2 remote_robot.launch
```

## PC side

```
roslaunch raspimouse_navigation_2 remote_desktop.launch
```

