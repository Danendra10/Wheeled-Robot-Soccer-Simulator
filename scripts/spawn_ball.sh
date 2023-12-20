#!/bin/bash

### spawn Bola
rosrun gazebo_ros spawn_model -file $(rospack find simulator)/models/bola/model.sdf -sdf \
	-model bola \
	-x 6.0 -y 4.0 -z 1.0

sleep 1