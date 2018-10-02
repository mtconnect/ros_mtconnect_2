#! /bin/sh
#Starting the single_robot_demo
xterm -e roslaunch ceccrebot_demo_support single_robot_demo.launch sim_robot:=true &

sleep 5
xterm -e roslaunch ceccrebot_demo robot.launch &


#Start Gripper
#xterm -e roslaunch ceccrebot_demo_support robotiq_gripper.launch &

sleep 2
#Starting the agent
xterm -e agent run /etc/mtconnect/agent/agent.cfg &

sleep 2
#Starting the mtconnect robot statemachine
xterm -e roslaunch ceccrebot_demo mtconnect.launch &

sleep 5
#Starting the cell.py

#xterm -e pipenv run python cnc.py &
#xterm -e pipenv run python cmm.py &
#xterm -e pipenv run python buffer.py &
#xterm -e pipenv run python cell.py &
xterm -e pipenv run idle -r cell.py &
