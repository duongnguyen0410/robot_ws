## Simulation

Run:

```
ros2 launch tibot2 launch_sim.launch.py world:=./src/tibot2/worlds/obstacles.world
```

## Raspi/robot_ws

Run:

```
sudo chmod 777 /dev/ttyUSB0 && ros2 launch tibot2 launch_robot.launch.py
```

---

## Dev machine/dev_ws

Run teleop:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Run Rviz:

```
ros2 run rviz2 rviz2
```

Run slam_toolbox:

```
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/tibot2/config/mapper_params_online_async.yaml use_sim_time:=false
```

Run Localization:

```
ros2 launch nav2_bringup localization_launch.py map:=./<map_file_save>.yaml use_sim_time:=false
```

**Change Topic/Durability Policy: Transient Local<br>
Then change fixed frame to map and set 2D Pose Estimate on Rviz**

Run Nav2:
**Change fixed frame to map when using Nav2**

```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient:=true
```

### _Default Rviz - current work space (i.e. tipa_ws)_

View motor motion:

```
ros2 run rviz2 rviz2 -d ./src/tibot2/rviz/main.rviz
```

View realsense:

```
ros2 run rviz2 rviz2 -d ./src/tibot2/rviz/realsense.rviz
```

View lidar:

```
ros2 run rviz2 rviz2 -d ./src/tibot2/rviz/rplidar.rviz
```
