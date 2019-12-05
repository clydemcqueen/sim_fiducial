# `sim_fiducial`

Shared models and scripts to simulate fiducial markers in Gazebo9.

[Install Gazebo-9](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

Add this ROS2 package to your workspace:
~~~
# Create workspace if this is the 1st package:
mkdir -p ~/sim_ws/src

# Add package:
cd ~/sim_ws/src
git clone https://github.com/clydemcqueen/sim_fiducial.git
cd ..

# Build:
. /opt/ros/eloquent/setup.bash
colcon build
. install/local_setup.bash
~~~

Set up the Gazebo environment:
~~~
export GAZEBO_MODEL_PATH=${PWD}/install/sim_fiducial/share/sim_fiducial/models
. /usr/share/gazebo/setup.sh
~~~

A sample launch:
~~~
ros2 launch sim_fiducial sample_launch.py
~~~
