#!/bin/bash
COM1='make px4_sitl gazebo_plane_cam__baylands'
COM2='roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"'
gnome-terminal  --tab --title="Gazebo" --command="bash -c 'cd PX4-Autopilot; $COM1 ; $SHELL'" --tab --title="Mavros" --command="bash -c '$COM2 ; ls; $SHELL'"