#!/bin/bash

jml_cyan=3
jml_magenta=3

cyan_x=(0.0 0.0 0.0 1.0)
cyan_y=(0.0 -1.0 0.0 -1.0)
magenta_x=(0.0 11.5 4.5 9.0)
magenta_y=(0.0 4.0 2.0 6.0)

sleep 1


### spawn cyan robots
for ((i=1; i<=$jml_cyan; ++i))
do
    rosrun gazebo_ros spawn_model -file $(rospack find simulator)/models/iris_cyan/model.sdf -sdf \
                                  -model cyan${i} \
                                  -x ${cyan_x[$i]} -y ${cyan_y[$i]} -z 0.1 &
    sleep 0.5
done 

## spawn magenta robots
for ((i=1; i<=$jml_magenta; ++i))
do
    rosrun gazebo_ros spawn_model -file $(rospack find simulator)/models/robot_magenta/model.sdf -sdf \
                                  -model magenta${i} \
                                  -x ${magenta_x[$i]} -y ${magenta_y[$i]} -z 1 &
    sleep 0.5
done 


## use joystick
rosrun joy joy_node &

