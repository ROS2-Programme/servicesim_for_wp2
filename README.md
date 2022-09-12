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

```bash
cd __INSTALL_PATH_OF_THIS_PACKAGE__
```

> Example 1

To generate test scenario 1 (i.e. red shirt actor walking length-wise around
upper row of three tables) at maximum walking speed (`c`):
```bash
xacro robot_name:=husky_ur5e custom_cfg:=1.c service.world.xacro |\
   grep -v -P "^<\/?robot>$" > /tmp/temp.world
```

> Example 2

To generate test scenario 2 (i.e. blue shirt actor walking breadth-wise around
middle column of two tables) at medium walking speed (`b`):
```bash
xacro robot_name:=husky_ur5e custom_cfg:=2.b service.world.xacro |\
   grep -v -P "^<\/?robot>$" > /tmp/temp.world
```

> Example 3

To generate test scenario 3 (i.e. green shirt actor walking zig-zag path back
and forth in main open space of PublicCafe) at minimum walking speed (`a`):
```bash
xacro robot_name:=husky_ur5e custom_cfg:=3.a service.world.xacro |\
   grep -v -P "^<\/?robot>$" > /tmp/temp.world
```

> Example 4

To generate test world combining all three scenarios above:
```bash
xacro robot_name:=husky_ur5e custom_cfg:=1.c,2.b,3.a service.world.xacro |\
   grep -v -P "^<\/?robot>$" > /tmp/temp.world
```

<!-- ======================================== -->
<a name="+S1.2"></a>
### 1.2. Launching manually generated Gazebo simulation world without robot

Assuming Gazebo simulation that was manually generated in
[Section 1.1](#+S1.1) is `/tmp/temp.world`, then run `roslaunch` as:

```bash
roslaunch servicesim_for_wp2 competition.launch \
  custom:=true \
  custom_prefix:=/tmp/temp
```

<!-- ======================================== -->
<a name="+S1.3"></a>
### 1.3. Launching manually generated Gazebo simulation world with robot

This section will require
[git@github.com:ROS2-Programme/ros2_programme_simulator.git](
ros2_programme_simulator) repository to have been set up and built, using
`cjl-devel` branch.

Assuming Gazebo simulation that was manually generated in
[Section 1.1](#+S1.1) is `/tmp/temp.world`, then run `roslaunch` as:

```bash
roslaunch servicesim_for_wp2 demo_world.launch \
  custom:=true \
  custom_prefix:=/tmp/temp
```

