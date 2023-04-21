# servicesim\_for\_wp2
Extension of servicesim\_competiition for customised simulation world for
testing, built using xacro files


<!-- ######################################## -->
<a name="+S1"></a>
## 1. Setup

<!-- ======================================== -->
<a name="+S1.1"></a>
### 1.1. 3rd-party package dependencies

<!-- ==================== -->
<a name="+S1.1.1"></a>
#### 1.1.1. `husky` repository (for `Kinetic` only)

**NOTE**:
- Source copy of `husky` repository **is not needed** for Noetic - just use
  the standard `ros-noetic-husky-*` packages that can be installed via
  `apt install` command.
- If it is really necessary to get a copy of `husky` repository source for
  Noetic, get the `${ROS_DISTRO}.zip` version rather than the
  `${ROS_DISTRO}-devel.zip` version - the latter's `husky_description` package
  contains URDF xacro macro names that are named differently from what is
  expected (e.g. no `intel_realsense_mount` macro in
  `husky_description/urdf/accesories/intel_realsense.urdf.xacro`).

Get Husky source (tested working for Kinetic) from official Clearpath
repository:
```bash
wget -O ~/Downloads/husky-${ROS_DISTRO}-devel.zip \
  https://github.com/husky/husky/archive/refs/heads/${ROS_DISTRO}-devel.zip
```

> **OR**:  
> Get Husky source (tested working for Noetic) from official Clearpath
> repository:
> ```bash
> wget -O ~/Downloads/husky-${ROS_DISTRO}.zip \
>   https://github.com/husky/husky/archive/refs/heads/${ROS_DISTRO}.zip
> ```

<!-- ==================== -->
<a name="+S1.1.2"></a>
#### 1.1.2. `husky_manipulation` repository

Clearpath provides a repository in which its Husky base robot description is
combined with a few popular robot manipulation arms into ready-to-use
description packages, with each one paired with a corresponding MoveIt config
package.

Get this Husky manipulation source (tested working for Kinetic & Noetic) from
the official Clearpath repository:
```bash
wget -O ~/Downloads/husky_manipulation-${ROS_DISTRO}-devel.zip \
  https://github.com/husky/husky_manipulation/archive/refs/heads/${ROS_DISTRO}-devel.zip
```

<!-- ==================== -->
<a name="+S1.1.3"></a>
#### 1.1.3. `velodyne-simulator` repository

Normal installation of `velodyne-simulator` package will be via standard `apt
install` command. E.g.:
```bash
sudo apt install ros-${ROS_DISTRO}-velodyne-simulator
```

However, if a different/newer version of Gazebo has been installed outside of
the standard Ubuntu repository (e.g. refer to [Install Gazebo using Ubuntu packages](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)),
in case the standard `apt install` of a package (that depends on the Gazebo
development library) may fail, complaining of a mismatched package dependecy.

In such cases, it may be necessary to build such packages that depend of the
Gazebo development library from source.
`velodyne-simulator` source code can be downloaded from:

[https://bitbucket.org/DataspeedInc/velodyne_simulator/downloads/](
https://bitbucket.org/DataspeedInc/velodyne_simulator/downloads/)


<!-- ==================== -->
<a name="+S1.1.4"></a>
#### 1.1.4. `gazebo-tutorials-velodyne` repository

This __should__ be an optional package.

It was probably used as a source reference for specifying a working SDF for
a simulated Velodyne sensor (in Gazebo), together with a plugin reference.

Copy of original repository source can be downloaded as:
```bash
wget -O ~/Downloads/gazebo-tutorials-velodyne-master.zip \
  https://github.com/dustycodes/gazebo-tutorials-velodyne/archive/refs/heads/master.zip
```

> Additional reference:  
> [Gazebo: Tutorial: Intermediate: Velodyne](http://classic.gazebosim.org/tutorials?tut=guided_i1)


<!-- ++++++++++++++++++++ -->
<!--
<a name="+S1.1.0a"></a>
##### 1.1.0a. `xxx` command
-->

<!-- ======================================== -->
<a name="+S1.2"></a>
### 1.2. In-house package dependencies

<!-- ==================== -->
<a name="+S1.2.1"></a>
#### 1.2.1. `servicesim_for_wp2_moveit_config` repository

This repository implements a MoveIt config package for implementing a
`move_group` for controlling the **arm only**, for supporting MVP0 run (see
[Section 4.1](#+S4.1)).

```bash
git clone https://git.i2r.a-star.edu.sg/fllai/servicesim_for_wp2_moveit_config
```

<!-- ==================== -->
<a name="+S1.2.2"></a>
#### 1.2.2. `mobile_manipulator_for_wp2_moveit_config_tray` repository

This repository implements a MoveIt config package for implementing a
`move_group` for controlling the **whole body**, for supporting MVP1a run (see
[Section 4.2](#+S4.2)).

```bash
git clone https://git.i2r.a-star.edu.sg/fllai/mobile_manipulator_for_wp2_moveit_config_tray
```

<!-- ======================================== -->
<a name="+S1.3"></a>
### 1.3. Build

<!-- ==================== -->

<mark>TODO</mark>


<!-- ######################################## -->
<a name="+S2"></a>
## 2. How to Run (`servicesim` Gazebo simulation only)

Current version doesn't yet support dynamic generation of `.world` file as
part of `roslaunch` command.

<!-- ======================================== -->
<a name="+S2.1"></a>
### 2.1. Manually generating custom `.world` file

**NOTE**:
- For all the example `xacro` commands shown below, you may set the value of
  the `have_gpu` argument to `0` (i.e. `False`) to disable use of Velodyne or
  Hokuyo lidar plugins that run with GPUs support. The default value of
  `have_gpu` is `1` (i.e. `True`), meaning that the GPU-enabled versions of
  the lidar plugins will be used by default.
- You also set the value of another optional argument `vis_lidar_2d` to `1`
  that will enable visualisation of the Hokuyo sensor's 2D lidar rays within
  Gazebo. There is also a corresponding optional argument `vis_lidar_3d` than
  can be similarly set to visualise the Velodyne sensor's 3D lidar rays within
  Gazebo (though the usefulness of this is debatable&hellip;).

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
    robot_name:=husky \
    custom_cfg:=1.c \
    service.world.xacro \
  |\
  grep -v -P "^<\/?robot\b.*>$" > /tmp/temp_single_red.world
```

- __Example 2__  
To generate test scenario 2 (i.e. blue shirt actor walking breadth-wise
around middle column of two tables) at medium walking speed (`b`):
```bash
xacro \
    use_skel_collision:=1 \
    have_gpu:=0 \
    robot_name:=husky \
    custom_cfg:=2.b \
    service.world.xacro \
  |\
  grep -v -P "^<\/?robot\b.*>$" > /tmp/temp_single_blue.world
```

- __Example 3__  
To generate test scenario 3 (i.e. green shirt actor walking zig-zag path
back and forth in main open space of PublicCafe) at minimum walking speed
(`a`):
```bash
xacro \
    use_skel_collision:=1 \
    have_gpu:=0 \
    robot_name:=husky \
    custom_cfg:=3.a \
    service.world.xacro \
  |\
  grep -v -P "^<\/?robot\b.*>$" > /tmp/temp_single_green.world
```

- __Example 4__  
To generate test world combining all three scenarios above:
```bash
xacro \
    use_skel_collision:=1 \
    have_gpu:=0 \
    robot_name:=husky \
    custom_cfg:=1.c,2.b,3.a \
    service.world.xacro \
  |\
  grep -v -P "^<\/?robot\b.*>$" > /tmp/temp_all.world
```

<!-- ======================================== -->
<a name="+S2.2"></a>
### 2.2. Launching manually generated Gazebo simulation world without robot

Assuming Gazebo simulation that was manually generated in
[Section 2.1](#+S2.1) is `/tmp/temp_all.world`, then run `roslaunch` as:

```bash
roslaunch servicesim_for_wp2 competition.launch \
  custom:=true \
  custom_prefix:=/tmp/temp_all
```

<!-- ======================================== -->
<a name="+S2.3"></a>
### 2.3. Launching manually generated Gazebo simulation world with robot

This section will require [ros2_programme_simulator](
#+git@github.com:ROS2-Programme/ros2_programme_simulator.git) repository to
have been set up and built, using `cjl-devel` branch.

Assuming Gazebo simulation that was manually generated in
[Section 2.1](#+S2.1) is `/tmp/temp_all.world`, then run `roslaunch` as:

```bash
roslaunch servicesim_for_wp2 demo_world.launch \
  custom:=true \
  custom_prefix:=/tmp/temp_all
```

<!-- ######################################## -->
<a name="+S3"></a>
## 3. Available plugin sensor ROS topics

<!-- ======================================== -->
<a name="+S3.1"></a>
### 3.1. Sensor output ROS topics for using for perception, prediction

Upon running either of the `roslaunch` commands in [Section 2.2](#+S2.2) or
[Section 2.3](#+S2.3), the following simulated sensor output topics should be
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
<a name="+S3.2"></a>
### 3.2. Quick command to view simulated RBG-D camera plugin output images

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

<!-- ######################################## -->
<a name="+S4"></a>
## 4. How to Run

<!-- ======================================== -->
<a name="+S4.1"></a>
### 4.1. For MVP0

```bash
HUSKY_LMS1XX_ENABLED=1 \
HUSKY_LMS1XX_TOPIC=scan \
HUSKY_URDF_EXTRAS=$(
  rospack find servicesim_for_wp2)/urdf/husky_ur5_e_for_wp2.urdf.xacro \
roslaunch servicesim_for_wp2_moveit_config demo_gazebo.launch \
  custom:=true \
  custom_prefix:=${HOME}/ROS2_WP/servicesim_for_wp2/test/test_single_green \
  custom_spawn:=true

roslaunch servicesim_for_wp2_moveit_config ros_controllers.launch

HUSKY_LMS1XX_ENABLED=1 \
HUSKY_LMS1XX_TOPIC=scan \
HUSKY_URDF_EXTRAS=$(
  rospack find servicesim_for_wp2)/urdf/husky_ur5_e_for_wp2.urdf.xacro \
roslaunch servicesim_for_wp2 spawn_husky.launch \
  x:=17.20 \
  y:=1.60 \
  z:=0.135 \
  yaw:=3.141593 \
  laser_enabled:=true

roslaunch servicesim_for_wp2 amcl_demo.launch \
  map_file:=${HOME}/ROS2_WP/servicesim_for_wp2/test/test_empty_180_wtf.yaml

rosrun rviz rviz -d $(rospack find servicesim_for_wp2)/test/test3.rviz

rosservice call /gazebo/unpause_physics

rosrun servicesim_for_wp2 test.py

rostopic echo --filter "(m.frame_id == 'human_94763') and (m.seq == 4)" -n 1 /gazebo/actor/way_pt; \
\
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
pose:
  position:
    x: 0.60
    y: -6.30
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0"; \
\
rostopic echo --filter "(m.status_list == []) or (m.status_list[0].status == 3)" -n 1 /move_base/status
```

> **NOTE**:  
> - `test.py` runs a `move_group` client that needs to connect to the
>   `move_group` service that is spawned as part of the `demo_gazebo.launch`
>   command, so `test.py` will fail to start up properly if run before the
>   `move_group` has fully started up in the `demo_gazebo.launch` command.  
>   To ensure `test.py` starts up successfully, wait until the
>   `demo_gazebo.launch` command terminal displays the standard
>   `You can start planning now!` message.
> - `demo_gazebo.launch` command does not perform all its initialisation steps
>   unless the ROS `/clock` topic is being published, which requires the
>   Gazebo simulator to be running, which can be achieved by running the
>   `rosservice` call to `/gazebo/unpause_physics`.
> - Arm movement to retract (to avoid collision of arm/tray with actor) and
>   then re-extend (to original pose) __currently needs to be manually
>   triggered__ (by pressing `ENTER` key in `test.py` command terminal), and
>   timed such that the action is synchronised with the mobile base's
>   trajectory as it passes close to the actor.

<!-- ======================================== -->
<a name="+S4.2"></a>
### 4.2. For MVP1a

```bash
HUSKY_LMS1XX_ENABLED=1 \
HUSKY_LMS1XX_TOPIC=scan \
HUSKY_URDF_EXTRAS=$(
  rospack find servicesim_for_wp2)/urdf/husky_ur5_e_for_mob_manip.urdf.xacro \
UR_EE_TYPE=tray \
WHOLE_BODY_PLAN_BY_MOVEIT=1 \
roslaunch mobile_manipulator_for_wp2_moveit_config_tray demo_gazebo.launch \
  custom:=true \
  custom_prefix:=$(rospack find servicesim_for_wp2)/test/test_single_green \
  custom_spawn:=true \
  pipeline:=ompl_add

HUSKY_LMS1XX_ENABLED=1 \
HUSKY_LMS1XX_TOPIC=scan \
HUSKY_URDF_EXTRAS=$(
  rospack find servicesim_for_wp2)/urdf/husky_ur5_e_for_mob_manip.urdf.xacro \
roslaunch servicesim_for_wp2 spawn_husky.launch \
  z:=0.135 \
  custom_joint_init:="-J robot_tx 17.20 -J robot_ty 1.60 -J robot_rz 3.141593" \
  laser_enabled:=true \
  bDoWtf:=true

roslaunch mobile_manipulator_for_wp2_moveit_config_tray ros_controllers.launch

rosservice call /gazebo/unpause_physics

rosrun rviz rviz -d $(rospack find servicesim_for_wp2)/test/wtf_mvp1e.rviz

rosrun servicesim_for_wp2 test_wtf.py
```

> **NOTE**:  
> - To allow mobile base to move without non-holonomic constraint, omit the
>   argument `pipeline:=ompl_add` in the roslaunch command for
>   `demo_gazebo.launch` above.

<!-- ======================================== -->
<a name="+S4.3"></a>
### 4.3. For MVP1b

<mark>TODO</mark>

