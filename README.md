# laser_scan_matcher_odometry

This repository is a fork from the official [laser_scan_matcher](http://wiki.ros.org/laser_scan_matcher) package in ROS.
Laser_scan_matcher_odometry offers an additional topic named **lsm_odom** to output ROS odometry messages.

![](launch/lsm_odom.gif)


## Usage

First, create a catkin_workspace:

<pre><code>
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
</pre></code>

clone this repository:

<pre><code>
cd ~/catkin_ws/src
git clone https://github.com/bierschi/laser_scan_matcher_odometry.git
</pre></code>

build with catkin_make:

<pre><code>
cd ~/catkin_ws/
catkin_make
</pre></code>

and finally run the example.launch file

<pre><code>
source devel/setup.bash
roslaunch laser_scan_matcher_odometry example.launch
</pre></code>