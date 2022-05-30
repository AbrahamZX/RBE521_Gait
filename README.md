RBE521 Readme.  woo-hoo

Github SSH key instructions:
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

To download into catkin_ws/src
cd catkin_ws
git clone git@github.com:AbrahamZX/RBE521_Gait.git src

Open terminal and run
roslaunch nao_gazebo empty_world.launch

Open second terminal and run
rosrun gazebo_ros spawn_model -file \`rospack find nao_description\`/naoV32_generated_urdf/nao.urdf -urdf -z 1 -model nao


Useful commands:
catkin build
source ~/catkin_ws/devel/setup.bash