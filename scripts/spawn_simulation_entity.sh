#!/bin/bash

VARITATION=2

# Variasi musuh 1 (holding the ball)
musuh_1_x=(2.0 2.0 2.0 2.0)  # x-coordinates for each variation
musuh_1_y=(7.5 7.5 9.0 5.5)  # y-coordinates for each variation

bola_1_y=(7.0 7.0 8.5 8.5)  # y-coordinates for each variation
bola_1_x=(2.0 2.0 2.0 6.0)

# Variasi musuh 2
musuh_2_x=(6.0 6.0 6.0 6.0)  # x-coordinates for each variation
musuh_2_y=(7.5 9.0 7.5 9.0)  # y-coordinates for each variation

jml_cyan=3

cyan_x=(0.0 0.5 0.0 0.0)
cyan_y=(0.0 4.0 0.0 8.0)

sleep 1

rosrun gazebo_ros spawn_model -file $(rospack find simulator)/models/bola/model.sdf -sdf \
	-model bola \
    -x ${bola_1_y[$VARITATION]} -y ${bola_1_x[$VARITATION]} -z 1.0

sleep 1

## spawn cyan robots
for ((i=1; i<=$jml_cyan; ++i))
do
    rosrun gazebo_ros spawn_model -file $(rospack find simulator)/models/iris_cyan/model.sdf -sdf \
                                  -model cyan${i} \
                                  -x ${cyan_x[$i]} -y ${cyan_y[$i]} -z 0.1 &
    sleep 0.5
done 

rosrun gazebo_ros spawn_model -file $(rospack find simulator)/models/robot_magenta/model.sdf -sdf \
                                  -model magenta1 \
                                  -x ${musuh_1_y[$VARITATION]} -y ${musuh_1_x[$VARITATION]} -z 1 &
sleep 0.5

rosrun gazebo_ros spawn_model -file $(rospack find simulator)/models/robot_magenta/model.sdf -sdf \
                                  -model magenta2 \
                                  -x ${musuh_2_y[$VARITATION]} -y ${musuh_2_x[$VARITATION]} -z 1 &
sleep 0.5