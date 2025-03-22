#!/bin/bash
cd ~/catkin_ws

sleep 1

source devel/setup.bash

sleep 1

rosrun corrosion_mapping trigger_servo_node
