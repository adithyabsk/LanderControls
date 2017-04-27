#!/bin/bash

sudo pppd -detach lock defaultroute 10.9.0.10:10.9.0.14 /dev/XBEE 115200 noauth persist &

sleep 5

. /home/ubuntu/LanderControls/lander_ws/ROSLander.sh

cd /home/ubuntu/LanderControls/lander_ws

roslaunch controller_node basic.launch




