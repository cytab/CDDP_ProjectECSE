# Name: TABE CYRILLE JAMABEL 

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

4. Use this to install [ros noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

## Install this repository with git clone 

## Now let's run some code

```
cd catkin_ws
```


```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```



```
export TURTLEBOT3_MODEL=burger
```

```
source devel/setup.bash 
```
5. The following 2 command must work ! (If not working terminate the terminal and redo the previous 2 commands in a new terminal)
```
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
```

```
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```


6. (optional) If you want to variate the initial position of the robot specify this command to the gazebo command :
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch x_pos:=x y_pos:=y z_pos=z
```

7. Before going forward you have to make some changes to the turtlebot package in order to facilitate the simulation. Go into the file catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/, open the file turtlebot3_empty_world.launch and change the whole file to : 
```
<launch>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="x_pos" default="0.0"/>
  <arg name="y_pos" default="0.0"/>
  <arg name="z_pos" default="0.0"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/multiple_obstacles.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch">
    <arg name="model" value="$(arg model)"/>
  </include>

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
   <node name="rviz" pkg="rviz" type="rviz" args="-d $(find turtlebot3_gazebo)/rviz/turtlebot3_gazebo_project.rviz"/>
</launch>

```

8. Now one problem will arise, you can remark in the above text, you need to have the turtlebot3_gazebo_project.rviz file in the right file. Change the place of the file to the empacement turtlebot3_gazebo/rviz/

```
1- Go into the project\src folder
2- move the file turtlebot3_gazebo_project.rviz to catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/rviz/ : mv -i turtlebot3_gazebo_project.rviz  ./../../turtlebot3_simulations/turtlebot3_gazebo/rviz/
```


9. Dive into src file created by doing cd src/ and create a file names project 
```
 cd project | cd src
```

10. Install the fast obstacles avoidance library by following the instruction [here](https://github.com/hubernikus/fast_obstacle_avoidance/tree/main)
For this library python 3.10 is mandatory otherwise you would need to comment some poart of their base code but it is not difficult, 


11. To run the project 2 essential command must be ran:

    1 The first command serve to initiate gazebo with the right environment (please run this command from the folder catkin_ws):
        ```
        roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
        ```
    2 This command initiate the mpc algorithm based on CDDP and modulation :
        ```
        python mpc_sim.py 
        ```
# What to expect when running 
1. Environment layout:
![Test Image 7](https://github.com/cytab/CDDP_ProjectECSE/tree/safe_Cddp_MCGILL_project)