# ros-mtconnect-2

A distributed digital manufacturing cell setup.

## Simulation Environment Setup

1. Clone ros_mtconnect_2 repository from MTConnect GitHub preferrably in the HOME directory.
    
    `cd ~`
    
    `git clone git@github.com:mtconnect/ros_mtconnect_2.git`
    
2. If repository is not cloned in the HOME directory, update the `src/` path in the `collaborationModel` package inside the `archetypeToInstance.py` module (ln 5) accordingly. 
    
3. (User Preference) Update the adapter host and port information for the devices in `cell.py` module and in `agent.cfg` file in the `configFiles` directory of the ros_mtconnect_2 repository.
    
4. Install the MTConnect Agent.
    
5. Follow instructions for installation and usage on 

    `https://github.com/mtconnect/cppagent`
    
6. Update adapter host, port and device file information for the devices in the `agent.cfg` of the MTConnect Agent as well. Or simply use `agent.cfg` file in the `configFiles` directory of the ros_mtconnect_2 repository.
    
7. Update the agent device file of the Agent to be `combined.xml` which can be found in `deviceFiles` directory.


## Python Environment Setup

Some of these steps might not be needed if the necessary python packages are already installed.

1. Open a terminal.

2. `sudo apt install python-pip`

3. `pip install pipenv`

4. `cd ~/ros_mtconnect_2/simulator`

     There should be a file \texttt{Pipfile} here

5. `pipenv install -d`
    
6. `gedit  ~/.bashrc`

7. Add this line to the bottom: 

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

