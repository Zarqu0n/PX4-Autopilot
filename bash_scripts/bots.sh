#!/bin/bash
COM1='bash test.sh'
COM2='python ~/PX4-Autopilot/src/control/bot1.py'
COM3='python ~/PX4-Autopilot/src/control/bot2.py'
COM4='python ~/PX4-Autopilot/src/control/bot3.py'
COM5='python ~/PX4-Autopilot/src/control/bot4.py'
COM6='python ~/PX4-Autopilot/src/control/bot5.py'
COM7='python ~/PX4-Autopilot/src/control/bot6.py'
gnome-terminal  --tab --title="PX4" --command="bash -c 'cd PX4-Autopilot/; $COM1; $SHELL'" --tab --title="BOT-1" --command="bash -c '$COM2; $SHELL'" --tab --title="BOT-2" --command="bash -c '$COM3; $SHELL'" --tab --title="BOT-3" --command="bash -c '$COM4; $SHELL'" --tab --title="BOT-4" --command="bash -c '$COM5; $SHELL'" --tab --title="BOT-5" --command="bash -c '$COM6; $SHELL'" --tab --title="BOT-6" --command="bash -c '$COM7; $SHELL'"