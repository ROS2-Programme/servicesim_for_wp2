# servicesim_for_wp2
Extension of servicesim_competiition for customised simulation world for
testing, built using xacro files


<!-- ######################################## -->
<a name="+S1"></a>
## 1. How to Run

Current version doesn't yet support dynamic generation of `.world` file as
part of `roslaunch` command.

<!-- ======================================== -->
<a name="+S1.1"></a>
### 1.1. Manually generating custom `.world` file

**NOTE**:
for all the example `xacro` commands shown below, you may set the value of
the `have_gpu` argument to `0` (i.e. `False`) to disable use of Velodyne or
Hokuyo lidar plugins that run with GPUs support. The default value of
`have_gpu` is `1` (i.e. `True`), meaning that the GPU-enabled versions of the
lidar plugins will be used by default.

```bash
cd __INSTALL_PATH_OF_THIS_PACKAGE__
```

- __Example 1__  
To generate test scenario 1 (i.e. red shirt actor walking length-wise around
upper row of three tables) at maximum walking speed (`c`):
```bash
xacro \
    use_skel_collision:=1 \
    have_gpu:=0 \
    robot_name:=husky_ur5e \
    custom_cfg:=1.c \
    service.world.xacro \
  |\
  grep -v -P "^<\/?robot>$" > /tmp/temp_single_red.world
```

- __Example 2__  
To generate test scenario 2 (i.e. blue shirt actor walking breadth-wise
around middle column of two tables) at medium walking speed (`b`):
```bash
xacro \
  use_skel_collision:=1 \
  have_gpu:=0 \
  robot_name:=husky_ur5e \
  custom_cfg:=2.b \
  service.world.xacro \
  |\
  grep -v -P "^<\/?robot>$" > /tmp/temp_single_blue.world
```

- __Example 3__  
To generate test scenario 3 (i.e. green shirt actor walking zig-zag path
back and forth in main open space of PublicCafe) at minimum walking speed
(`a`):
```bash
xacro \
  use_skel_collision:=1 \
  have_gpu:=0 \
  robot_name:=husky_ur5e \
  custom_cfg:=3.a \
  service.world.xacro \
  |\
  grep -v -P "^<\/?robot>$" > /tmp/temp_single_green.world
```

- __Example 4__  
To generate test world combining all three scenarios above:
```bash
xacro \
  use_skel_collision:=1 \
  have_gpu:=0 \
  robot_name:=husky_ur5e \
  custom_cfg:=1.c,2.b,3.a \
  service.world.xacro \
  |\
  grep -v -P "^<\/?robot>$" > /tmp/temp_all.world
```

<!-- ======================================== -->
<a name="+S1.2"></a>
### 1.2. Launching manually generated Gazebo simulation world without robot

Assuming Gazebo simulation that was manually generated in
[Section 1.1](#+S1.1) is `/tmp/temp_all.world`, then run `roslaunch` as:

```bash
roslaunch servicesim_for_wp2 competition.launch \
  custom:=true \
  custom_prefix:=/tmp/temp_all
```

<!-- ======================================== -->
<a name="+S1.3"></a>
### 1.3. Launching manually generated Gazebo simulation world with robot

This section will require [ros2_programme_simulator](
#+git@github.com:ROS2-Programme/ros2_programme_simulator.git) repository to
have been set up and built, using `cjl-devel` branch.

Assuming Gazebo simulation that was manually generated in
[Section 1.1](#+S1.1) is `/tmp/temp_all.world`, then run `roslaunch` as:

```bash
roslaunch servicesim_for_wp2 demo_world.launch \
  custom:=true \
  custom_prefix:=/tmp/temp_all
```

<!-- ######################################## -->
<a name="+S2"></a>
## 2. Available plugin sensor ROS topics

<!-- ======================================== -->
<a name="+S2.1"></a>
### 2.1. Sensor output ROS topics for using for perception, prediction

Upon running either of the `roslaunch` commands in [Section 1.2](#+S1.2) or
[Section 1.3](#+S1.3), the following simulated sensor output topics should be
available for each of the 5 cameras added to the simulation world:

- RGB-D camera (`libgazebo_ros_openni_kinect.so`)  
```bash
/camera<N>/color/camera_info
/camera<N>/color/image_raw
/camera<N>/color/image_raw/compressed
/camera<N>/color/image_raw/compressed/parameter_descriptions
/camera<N>/color/image_raw/compressed/parameter_updates
/camera<N>/color/image_raw/compressedDepth
/camera<N>/color/image_raw/compressedDepth/parameter_descriptions
/camera<N>/color/image_raw/compressedDepth/parameter_updates
/camera<N>/color/image_raw/theora
/camera<N>/color/image_raw/theora/parameter_descriptions
/camera<N>/color/image_raw/theora/parameter_updates
/camera<N>/depth/camera_info
/camera<N>/depth/image_raw
/camera<N>/depth/points
/camera<N>/parameter_descriptions
/camera<N>/parameter_updates
```
> __where__: `<N>` will range from 1 to 5.

- Hokuyo 2D lidar (`libgazebo_ros_laser.so` or `libgazebo_ros_gpu_laser.so`)  
```bash
/hokuyo1/hokuyo_points
```

- Velodyne 3D lidar (`libgazebo_ros_velodyne_laser.so` or
  `libgazebo_ros_velodyne_gpu_laser.so`)  
```bash
/velodyne1/velodyne_points
```


<!-- ======================================== -->
<a name="+S2.2"></a>
### 2.2. Quick command to view simulated RBG-D camera plugin output images

Use standard `image_view` ROS package to view the simulated camera images from
the command prompt.

- __Example 5__  
To view RBG image of simulated camera 2:
```bash
rosrun image_view image_view image:=/camera2/color/image_raw
```

- __Example 6__  
To view depth image of simulated camera 3:
```bash
rosrun image_view image_view image:=/camera3/depth/points
```

