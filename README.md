# uwb_sim
uwb simulation on ROS2 Humble and Gazebo Garden.

## Ho to Use

### 1 Gazebo Plugin

inset the plugin `gz-sim-odometry-publisher-system` in gazebo model SDF file.

```
<sdf version='1.9'>
  <model name='x500'>

    ...

    <plugin
    filename="gz-sim-odometry-publisher-system"
    name="gz::sim::systems::OdometryPublisher">
    <odom_publish_frequency>50</odom_publish_frequency>
    <dimensions>3</dimensions>
    <!-- <odom_topic>odom</odom_topic> -->
    <odom_frame>odomCustom</odom_frame>
    <robot_base_frame>baseCustom</robot_base_frame>
    </plugin>
    </model>
</sdf>
```
The odometry data will publisher to the gazebo topic.

Such as `/model/x500_1/odometry`.

You can use `gz topic -e -t /model/x500_2/odometry` to check out the msgs.

```
header {
  stamp {
    sec: 357
    nsec: 480000000
  }
  data {
    key: "frame_id"
    value: "odomCustom"
  }
  data {
    key: "child_frame_id"
    value: "baseCustom"
  }
}
pose {
  position {
    x: 4.0537175486223009e-17
    y: 5.4769187601530667e-16
    z: 0.22699981437144295
  }
  orientation {
    x: -5.4229149552466766e-17
    y: -7.6264627553662089e-17
    z: 1.2771610713269328e-18
    w: 1
  }
}
twist {
  linear {
  }
  angular {
  }
}

```

### 2 UWB Simulation

First edit UWB author config file in `config/anthor.xml`.

Build the UWB Simulation program.

```
source /opt/ros/humble/setup.sh
make sim
```

Run the UWB Simulation program.

```
source /opt/ros/humble/setup.sh
source install/setup.sh
ros2 run uwb_sim uwb_simulation
```

The defult label name is `x500_1`.

You can use parameter to assign.

```
ros2 run uwb_sim uwb_simulation --ros-args -p label_name:=x500_2
```

Program will output real distance and sim distance.

```
[INFO] [1676978364.181314890] [uwb_simulation]: position: 0.000000 0.000000 0.227000
[INFO] [1676978364.181387268] [uwb_simulation]: label name: x500_2 anthor Id: 1 real distance : 0.227000 sim distance : 0.316312.
[INFO] [1676978364.181435666] [uwb_simulation]: label name: x500_2 anthor Id: 2 real distance : 10.002576 sim distance : 10.048888.
[INFO] [1676978364.181452126] [uwb_simulation]: label name: x500_2 anthor Id: 3 real distance : 10.002576 sim distance : 10.121775.
[INFO] [1676978364.181464859] [uwb_simulation]: label name: x500_2 anthor Id: 4 real distance : 14.143957 sim distance : 14.061946.
[INFO] [1676978364.181477146] [uwb_simulation]: label name: x500_2 anthor Id: 5 real distance : 18.029184 sim distance : 18.150420.
[INFO] [1676978364.181489173] [uwb_simulation]: label name: x500_2 anthor Id: 6 real distance : 25.001030 sim distance : 24.788486.
[INFO] [1676978364.181501873] [uwb_simulation]: label name: x500_2 anthor Id: 7 real distance : 25.001030 sim distance : 25.150981.
```
