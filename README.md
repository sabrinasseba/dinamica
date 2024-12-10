<p align="center">
  <img width="400" height="400" src="https://github.com/user-attachments/assets/936c0e67-352a-479a-b908-540a19837f53">
</p>

# Fury Forge Mechanics

<p align="justify">

The Fury Forge Mechanics group is responsible for developing a robotic manipulator capable of performing welds.
</p>

<p align="justify">
  
The project concept was created for the course *Dynamics of Robotic Systems*, taken at the University of São Paulo and taught by Professor [Marcelo Becker](https://www.linkedin.com/in/marcelo-becker-761bb524/). The objective of the project was to design a robotic manipulator that served a purpose chosen by the students themselves. To achieve this, we not only came up with a use case idea but also carried out the entire development of the technical areas (mechanics, electronics, and programming) as well as the business development aspect.
</p>

# Setting Up

<p align="justify">

This work was made using Docker containers, and is set up with ROS 2 Humble.

What is included in this repository?
* Dockerfile with the required build instructions.
* Scripts that make docker easier to use.
* Description files for the manipulator.
* Launch files to run the simulation.

</p>

## Step 1 - Cloning the git repository

To download the repository execute the following command.
</p>

```bash
git clone https://github.com/sabrinasseba/dinamica.git
```

## Step 2 - Building Docker Container

<p align="justify">

The following command will build the required docker image with ROS Humble distribution and some additional packages.
</p>

*Inside the "dinamica" folder on your computer.

```bash
docker/scripts/build.sh 
```

## Step 3 - Running Docker Container

<p align="justify">

The command bellow, using the script **run.sh**, will execute the desired image and inicialize the container with a working ROS, ready to use.
</p>

```bash
docker/scripts/run.sh
```

## Step 4 - Setting up ROS Workspace

<p align="justify">

To build your workspace it is necessary to execute the commands bellow.

*Already inside the container.

```
colcon build
```
```
source install/setup.bash
```
## Step 5 - Runnig the simulation

<p align="justify">

To run the ROS simulation, you need to navigate to the package **robot_description**. To do so, write the command bellow.
</p>

```
cd robot_description
```
<p align="justify">

This will take you to the package containing the **config**, **launch**, **meshes**, **rviz2**, and **urdf** folders. Inside this directory, you can review the construction of the xacro/urdf files, launch files, and the meshes used.
</p>

<p align="justify">

To run the simulation on Rviz of the manipulator, just go back to your initial directory and use the following command.

```
ros2 launch robot_description display.launch.py
```







