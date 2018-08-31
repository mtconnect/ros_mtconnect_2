#Starting the single_robot_demo
#! /bin/sh
source ~/catkin_workspace/devel/setup.sh
xterm -e roslaunch ceccrebot_demo_support single_robot_demo.launch &

sleep 5
xterm -e roslaunch ceccrebot_demo robot.launch &


#Starting the agent
xterm -e agent run /etc/mtconnect/agent/agent.cfg &


sleep 10
#Starting the mtconnect robot statemachine

xterm -e roslaunch ceccrebot_demo mtconnect.launch &

sleep 3
#Starting the cell.py

#xterm -e pipenv run python cnc.py &
#xterm -e pipenv run python cmm.py &
#xterm -e pipenv run python buffer.py &
#xterm -e pipenv run python cell.py &
xterm -e pipenv run idle -r cell.py &
