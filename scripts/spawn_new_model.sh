#!/bin/bash

rosrun gazebo_ros spawn_model   -file $(rospack find iris_robot_description)/urdf/iris_robot.sdf \
                                -sdf -model cyan1 -x -5.5 -y 0 -z 0 -R 0 -P 0 -Y 2.34
