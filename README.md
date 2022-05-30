RBE521 Readme.  woo-hoo

Open terminal and run
roslaunch nao_gazebo empty_world.launch

Open second terminal and run
rosrun gazebo_ros spawn_model -file `rospack find nao_description`/naoV50_generated_urdf/nao.urdf -urdf -z 1 -model nao