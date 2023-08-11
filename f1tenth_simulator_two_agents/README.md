# [DISSERTATION] F1TENTH Simulator Two-Agents


> This repository is the first part of my dissertation "[F1TENTH](https://f1tenth.org/index.html): Development of A Multi-Agent Simulator and An Overtaking Algorithm Using Machine Learning".  
> This is a lightweight 2D simulator of [F1TENTH Racecar](https://f1tenth.org/index.html). It based on Robot Operating System, and using Rviz as visualization tool. This simulator has 2D-LiDAR simulation, collision checker, vehicle model, planning algorithm, and other features.

Please feel free to raise topics in Issues section, I will try my best to answer them!


## Environment


### Software


- Operating System: Ubuntu 20.04 (amd64/arm64)
- Robot Operating System: ROS Noetic (desktop-full)     
    
If you want to use overtaking algorithm:   
> [Depends on your GPU](https://www.tensorflow.org/install/pip#hardware_requirements), there are two different ways of installing tensorflow. If your GPU has cuda cores, you can install from pip. If not, you must build from source. Moreover, you can install tensorflow and other python packages in a Miniconda env, but you must activate that environment before launch the simulator in a new terminal every time.
- tensorflow 2.10.0 (I think 2.9.0/2.8.0/2.7.0 should also work)
- numpy 1.22.4

If you want to use MPC algorithm:

- casadi 3.5.5
- numpy 1.22.4
- pandas 1.4.2


### Hardware (recommended)


> If you are using virtual machine, better using a virtual machine that support GPU virtualization, or you can dual-boot. I am using Parallels VM and developed everything in this virtual machine, no performance issue with M1 MAX.

- dual-boot  
CPU: 4 cores with 3GHz  
GPU: GTX 1660  
RAM: 8GB

- virtual machine   
CPU: 8 cores    
GPU: RTX 2080   
RAM: 16GB   

## Install for first time

If you have ```ros-noetic-desktop-full``` installed, the additional dependencies you must install are:

- tf2_geometry_msgs
- ackermann_msgs
- joy
- map_server

You can install them by running:

    sudo apt-get install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-map-server


To install the simulator package, clone the repository with the simulator and starter code into your catkin workspace by running command below:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/JZ76/f1tenth_simulator_two_agents.git
    
Then run ```catkin_make``` to build it:

    cd ~/catkin_ws
    catkin_make


## Every time open the simulator in a new terminal


To run the simulator in a new terminal, run:

    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch f1tenth_simulator_two_agents simulator.launch

> Note: after run `cd ~/catkin_ws` and `source devel/setup.bash` commands, you can keep that terminal opening, so that you don't have to run these two commands before launch the simulator every time.


## Every time you edit some code in C++ files


To make your changes works, you need to build the simulator again, by using `catkin_make`:

    catkin_make

**Summary: Once you sourced a terminal, there are only two commands used frequently, one is `catkin_make` when you did some changes, another is `roslaunch f1tenth_simulator simulator.launch` to open the simulator**


## Uninstall

    cd ~/catkin_ws/src
    rm -rf f1tenth_simulator_two_agents
    cd ..
    catkin_make
    
You also need to manually delete `f1tenth_simulator_two_agents` dictionary in `~/catkin_ws/build`


## Usage


Please see [Wiki section](https://github.com/JZ76/f1tenth_simulator_two_agents/wiki) for instruction in detail




