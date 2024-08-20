# RL_Crazyflie

Here is a ROS2 package to implement an RL policy on Craziflie2.x with data acquisition from VICON.

**Note:** On each new terminar is needed to source ROS2 and the installation:
```shell
$ ros
$ source install/setup.zsh
```

1. Launch crazyswarm - _terminal 0_
    ```shell
    $ ros2 launch crazyflie launch.py
    ```
2. Launch the docker container - _terminal 1_
    ```shell
    $ cd ~/crazyfly_vicon_docker/
    $ zsh run_docker_container.sh
    ```
3. Export the ros1 dependencies for bridge - _terminal 2_
    ```shell
    $ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/fausto/ros_catkin_ws/install_isolated/lib
    ```
   run ros1_bridge
    ```shell
    $ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    ```
4. Run the CrazyflieRLControl script - _terminal 3_
   ```shell
   $ ros2 run rlcrazyflie CrazyflieRLControl
   ```
5. Start the engines - _terminal 4_: It needs to run exactly once, use CTRL + C, to stop it.
   ```shell
   $ ros2 topic pub /startfly std_msgs/String "{}"
   ```
6. Stop the engines - _terminal 5_: This is used in case the drone flips.
   ```shell
   $ ros2 topic pub /stopfly std_msgs/String "{}"
   ```
7. If the drone if flying and needs to land, runs - _terminal 6_:
   ```shell
   $ ros2 topic pub /landing std_msgs/String "{}"
   ```