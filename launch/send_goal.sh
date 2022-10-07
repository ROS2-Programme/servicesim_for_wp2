#! /usr/bin/env bash

## ----------------------------------------
function wait_for_goal_done() {
  local _topic="/move_base/status"

  rostopic echo --filter "(m.status_list == []) or (m.status_list[0].status == 3)" -n 1 ${_topic}
}

## ----------------------------------------
function euler_2_quaternion() {
  declare -n _pRet="$1"
  local _yaw="0"
  local _prec=8

  if [ $# -ge 2 ]; then
    _yaw="$2"
  fi

  _pRet=(`python3 -c "
import math;
from tf.transformations import quaternion_from_euler;
q = quaternion_from_euler( 0, 0, ${_yaw}/180.0*math.pi); 
print( round( q[2], ${_prec}), round( q[3], ${_prec}));
"`)
}

## ----------------------------------------
function set_simple_goal() {
  local _topic="/move_base_simple/goal"
  local _msg_type="geometry_msgs/PoseStamped"
  local _frame="odom"
  local _x="$1"
  local _y="$2"
  local _oZ="$3"
  local _oW="$4"

  # echo $#

  if [ $# -ge 3 ]; then
    # echo "\$3 = \"$3\""
    _yaw="$3"
  fi

  rostopic pub -1 ${_topic} ${_msg_type} "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '${_frame}'
pose:
  position:
    x: ${_x}
    y: ${_y}
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: ${_oZ}
    w: ${_oW}"
}


## ------------------------------------------------------------
function run_test() {

  # set_simple_goal 20.15 1.45 90
  # set_simple_goal 20.15 1.45 0
  # set_simple_goal 20.15 1.45 180

  local _pOzw=()

  euler_2_quaternion "_pOzw" 90
  echo ${_pOzw[@]}
  euler_2_quaternion "_pOzw" 0
  echo ${_pOzw[@]}
  euler_2_quaternion "_pOzw" 180
  echo ${_pOzw[@]}
  euler_2_quaternion "_pOzw" -90
  echo ${_pOzw[@]}
  euler_2_quaternion "_pOzw" -180
  echo ${_pOzw[@]}
}


## ------------------------------------------------------------
function run() {

  wait_for_goal_done

  euler_2_quaternion "_pOzw" 0
  set_simple_goal 3.0 -0.1 ${_pOzw[@]}
  wait_for_goal_done

  euler_2_quaternion "_pOzw" -180
  set_simple_goal 3.55 -6.45 ${_pOzw[@]}
  wait_for_goal_done
}

# run_test
run

