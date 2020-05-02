# gazeboSimulation

File paths look for repo (gazeboSimulation) it should be cloned in catkin_ws/src/

add the following to ~/.bashrc
```

export GAZEBO_MODEL_PATH=(path to repo)/models:$GAZEBO_MODEL_PATH

export GAZEBO_RESOURCE_PATH=(path to repo):$GAZEBO_RESOURCE_PATH
```

An example
```

export GAZEBO_MODEL_PATH=/home/tabor/catkin_ws/src/gazeboSimulation/models:$GAZEBO_MODEL_PATH

export GAZEBO_RESOURCE_PATH=/home/tabor/catkin_ws/src/gazeboSimulation:$GAZEBO_RESOURCE_PATH
```

Remember to open new terminal or run `source ~/.bashrc` after adding commands
