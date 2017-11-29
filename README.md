# gazeboSimulation
All the files and instructions you need to simulate Vex In The Zone field and parts in Gazebo

Create ros package named gazebosim and save git repo inside.

add the following to ~/.bashrc

source <install path>/gazebo/setup.sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/gazebosim/gazeboSimulation/models
export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/gazebosim/gazeboSimulation/worlds

**If you already have a model or resource path specified, separate with colons
