# ros-mtconnect-2

A distributed digital manufacturing cell setup.

## Simulation Environment Setup

1. Clone ros_mtconnect_2 repository from MTConnect GitHub preferrably in the HOME directory.
    
    `cd ~`
    
    `git clone https://github.com/mtconnect/ros_mtconnect_2.git`
    
2. Update the `src/` path in the `collaborationModel` package inside the `archetypeToInstance.py` module (ln 5) accordingly.
    
3. (User Preference) Update the adapter host and port information for the devices in `cell.py` module and in `agent.cfg` file in the `configFiles` directory of the ros_mtconnect_2 repository.
    
4. Install the MTConnect Agent.
    
5. Follow instructions for installation and usage on 

    `https://github.com/mtconnect/cppagent`
    
6. Update adapter host, port and device file information for the devices in the `agent.cfg` of the MTConnect Agent as well. Or simply use `agent.cfg` file in the `configFiles` directory of the ros_mtconnect_2 repository with an updated `Devices` file path (ln 1).
    
7. Update the agent device file of the Agent to be `combined.xml` which can be found in `deviceFiles` directory.


## Python Environment Setup

Some of these steps might not be needed if the necessary python packages are already installed.

1. Open a terminal.

2. `sudo apt install python-pip`

3. `pip install pipenv`

4. `cd ~/ros_mtconnect_2/simulator`

     There should be a file `Pipfile` here

5. `pipenv install -d`

6. `pipenv shell`

	This will launch the pip environment in the terminal
	
7. `pip install transitions doublex requests mock mamba`

8. Open a new terminal and in that terminal type:

	`killall pipenv`
    
9. `gedit  ~/.bashrc`

10. Add this line to the bottom: 

    `export PYTHONPATH=\$PYTHONPATH:/usr/lib/python2.7/dist-packages`


## Unit Testing

The setup can be tested before being run. Following are the steps to run the spec tests developed for the simulator behavior.

1. Make sure that necessary python dependencies are installed.

2. Goto the `spec/` directory.

    `cd ~/ros_mtconnect_2/simulator/spec`
    
    use as is if repository cloned in the HOME directory.

3. Run:
    
    `mamba *`
    
    runs all the spec tests available for the different devices, collaboration models and interfaces state-machines and their behaviors.
 

## Simulation

To run the simulation follow the following steps:

1. Go to the `src/` directory of the `simulator/`.

2. `cd ~/ros_mtconnect_2/simulator/src`

3. Run the MTConnect Agent with the updated configuration.
    
    `agent run ~/ros_mtconnect_2/simulator/src/configFiles/agent.cfg &`

4. Run the `cell.py` module as follows:
    
    `python cell.py`


## Simulation in ROS Environment with RViz

Check out `https://github.com/mtconnect/ros_mtconnect_2_demo`


## License

Copyright 2018, VIMANA. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
