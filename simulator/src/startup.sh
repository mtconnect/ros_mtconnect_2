#Starting the single_robot_demo
#! /bin/sh
source ~/catkin_workspace/devel/setup.sh
xterm -e roslaunch ceccrebot_demo_support single_robot_demo.launch &

sleep 5
xterm -e roslaunch ceccrebot_demo robot.launch &


#Starting the agent
xterm -e agent run /etc/mtconnect/agent/agent.cfg &


sleep 7
#Starting the mtconnect robot statemachine

xterm -e roslaunch ceccrebot_demo mtconnect.launch &

sleep 3
#Starting the cell.py
source ~/catkin_workspace/src/ceccrebot/simulator/src/
xterm -e pipenv run python cell.py 
