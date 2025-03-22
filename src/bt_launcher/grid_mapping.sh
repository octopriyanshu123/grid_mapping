#!/bin/bash
cd ~/catkin_ws

sleep 1

source devel/setup.bash

sleep 1

rosrun grid_mapping bt_lac_zero_node
