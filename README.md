# SwarmManipulation

## Clone repo

Clone the repo in the `src` folder of the ros2 workspace.
```
cd ros2_ws/src/
git clone git@github.com:fbenti/SwarmManipulation.git --recursive
git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git
```

## Build
```
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Launch

Change the [crazyflie.yaml](crazyswarm2/crazyflie/config/crazyflies.yaml) file according to your need, then launch the server. There are different parameters that you can define from the command line. Check the [launch.py](crazyswarm2/crazyflie/launch/launch.py) to find out more about it.

```
ros2 launch crazyflie launch.py
```

Once the server is up and running, you can launch the [OrchestratorNode](crazyflie_traj/crazyflie_traj/orchestratorNode.py) from the [crazyflie_traj](crazyflie_traj) pkg. Basically, this node is a high-level controller for the crazyflies' trajectories, which should be executed in a syncronized way. 

```
ros2 run crazyflie_traj orchestrator
```


## Update submodule

This is an example of how correclty push updates to the submodule:

```
cd crazyswarm2/
git commit -a -m "Pushing new commit"
git push
cd ..
git add crazyswarm2
git commit -m "Updated submodule"
```

## Further Info

For more information check the [official Crazyswarm2 website](https://imrclab.github.io/crazyswarm2/index.html).