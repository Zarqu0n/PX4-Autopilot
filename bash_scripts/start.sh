#!/bin/bash
COM1='sh ~/Scripts/Gazebo_plane/conf.sh'
COM2='sh ~/Scripts/Gazebo_plane/img.sh'
gnome-terminal --geometry=800x800 --title="HUMA TAKIMI" --command="bash -c '$COM2;sleep 2'"
sleep 2
gnome-terminal --title="Yer istasyonu" --command="bash -c ' $COM1 ;./Documents/GroundControl.AppImage; $SHELL'"
kill -9 $PPID