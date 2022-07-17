# Nao Gait
RBE 521 Project with a Nao V40 robot that is executing a gait in a gazebo simulation

Github SSH key instructions:
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

In order to run this ROS directory the requirements are being on Linux 20.04 and having ROS Noetic.
For instructions on how to install ROS Noetic and setup a workspace directory, consult this link:
http://wiki.ros.org/noetic/Installation/Ubuntu

For instructions on how to setup a Github SSH key, consult this link:
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

Instructions on how to clone the repo to a local catkin workspace directory:

1. Navigate to the ROS workspace directory in a terminal (usually "catkin_ws/src").

2. Inside this directory execute:
```
git@github.com:AbrahamZX/RBE521_Gait.git
```

With a copy of the repo in the catkin worspace folder now build and source by executing the follwing in a terminal inside of 
the catkin_ws directory:
```
catkin_make
```
```
source devel/setup.bash
```

Now it is possible to run the gazebo simulation and the python script that moves the robot's legs by doing the following:

1. Open two terminals, one can be left on the default directory and the other will be navigated towards the following 
directory inside of the cloned repo files by doing ("catkin_ws" can be replaced by your catkin workspace directory name):
```
cd ~/catkin_ws/src/RBE521_Gait/nao_virtual/nao_gazebo_plugin/scripts
```

2. On the first terminal, start the simulation of the nao robot by executing:
```
roslaunch nao_gazebo_plugin nao_gazebo.launch
```

3. Once the nao simulation is running, click on "Physics" in the "World" tab on the left, and uncheck "enable physics".

4. Now, run the python script on the second terminal inside of the "scripts" folder previously navigated, by executing:
```
python3 walk_sim.py
```
