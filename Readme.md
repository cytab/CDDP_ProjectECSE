# Installation

These installation instructions are created for Ubuntu 20.04. If you are using a different OS you made need to make some changes to the installing instructions. 


## Ubuntu 20.04
## Python 3.9 or 3.10

## Install Python Environment


1. Install conda, if you don't already have it, by following the instructions at [this link](https://docs.conda.io/projects/conda/en/latest/user-guide/install/)

This install will modify the `PATH` variable in your bashrc.
You need to open a new terminal for that path change to take place (to be able to find 'conda' in the next step).

2. Create a conda environment that will contain python 3:
```
conda create -n safe_cddp python=3.10
```
3. Activate python environment

```
conda activate safe_cddp
```

## Install ROS Noetic 

Use this to install [ros noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

## Install this repository with git clone 

## Now let's run some code

```
cd catkin_ws
```


```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```


```
source devel/setup.bash 
```

```
export TURTLEBOT3_MODEL=burger
```

```
source devel/setup.bash 
```
```
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
```

```
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```
```
python mpc_sim
```

If you want to variate the initial position of the robot specify this command to the gazebo command :
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch x_pos:=x y_pos:=y z_pos=z
```

Dive into src file created by doing cd src/ and create a file names project 
```
 cd project | cd src
```


Install the fast obstacles avoidance library by following the instruction [here](https://github.com/hubernikus/fast_obstacle_avoidance/tree/main)
For this library python 3.10 is mandatory otherwise you would need to comment some poart of their base code but it is not difficult, 

