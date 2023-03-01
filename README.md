# uwb_sim
UWB Simulation on ROS2 Humble and Gazebo Garden.

## Install

Install ROS2 Humble and Gazebo Garden.

Download th code.

```
git clone https://github.com/vacabun/uwb-simulation-system.git
git submodule update --init --recursive
```

## How to Use

### 1 Gazebo Plugin

Insert the plugin `gz-sim-odometry-publisher-system` in gazebo model SDF file.

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

Such as `/model/x500_0/odometry`.

You can use `gz topic -e -t /model/x500_0/odometry` to check out the msgs.

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

First edit UWB author config file in `config/anchor.xml`.

Build the UWB Simulation program.

```
source /opt/ros/humble/setup.sh
make sim
```

Run the UWB Simulation program.

```
source /opt/ros/humble/setup.sh
source install/setup.sh
ros2 run uwb_simulation uwb_simulation_node
```

The defult label name is `x500_0`.

You can use parameter to assign.

```
ros2 run uwb_simulation uwb_simulation_node --ros-args -p label_name:=x500_1
```

Program will output real distance and sim distance.

```
[INFO] [1676978364.181314890] [uwb_simulation_node]: position: 0.000000 0.000000 0.227000
[INFO] [1676978364.181387268] [uwb_simulation_node]: label name: x500_2 anchor Id: 1 real distance : 0.227000 sim distance : 0.316312.
[INFO] [1676978364.181435666] [uwb_simulation_node]: label name: x500_2 anchor Id: 2 real distance : 10.002576 sim distance : 10.048888.
[INFO] [1676978364.181452126] [uwb_simulation_node]: label name: x500_2 anchor Id: 3 real distance : 10.002576 sim distance : 10.121775.
[INFO] [1676978364.181464859] [uwb_simulation_node]: label name: x500_2 anchor Id: 4 real distance : 14.143957 sim distance : 14.061946.
[INFO] [1676978364.181477146] [uwb_simulation_node]: label name: x500_2 anchor Id: 5 real distance : 18.029184 sim distance : 18.150420.
[INFO] [1676978364.181489173] [uwb_simulation_node]: label name: x500_2 anchor Id: 6 real distance : 25.001030 sim distance : 24.788486.
[INFO] [1676978364.181501873] [uwb_simulation_node]: label name: x500_2 anchor Id: 7 real distance : 25.001030 sim distance : 25.150981.
```

You can use `ros2 topic echo /uwbData` to check out the msgs.

```
label_name: x500_2
distances:
- anchor_id: 1
  label_name: x500_2
  distance: 0.30048777956045086
- anchor_id: 2
  label_name: x500_2
  distance: 10.00039584047096
- anchor_id: 3
  label_name: x500_2
  distance: 10.16230345595602
- anchor_id: 4
  label_name: x500_2
  distance: 14.174792443625604
- anchor_id: 5
  label_name: x500_2
  distance: 17.98800797337829
- anchor_id: 6
  label_name: x500_2
  distance: 24.888237736000235
- anchor_id: 7
  label_name: x500_2
  distance: 24.97134236567964
---
```

### 3 UWB Locate

install the GSL libraries.

```
sudo apt-get install libgsl-dev
```

Build the UWB locate program.

```
source /opt/ros/humble/setup.sh
make locate
```

Run the UWB locate program.

```
source /opt/ros/humble/setup.sh
source install/setup.sh
ros2 run uwb_locate uwb_location
```

Program will output locate res.

```
[INFO] [1676981683.427636233] [uwb_location]: label: x500_2 uwb location success. res: x: -0.083882, y: 0.504697
[INFO] [1676981683.448672566] [uwb_location]: label: x500_2 uwb location success. res: x: -0.338217, y: 1.111799
[INFO] [1676981683.468478988] [uwb_location]: label: x500_2 uwb location success. res: x: -0.144554, y: 0.067628

```
