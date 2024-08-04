#!/bin/bash

cd /home/bigdavid/Nav2 
source install/setup.bash 
echo "load..." 

sleep 5 

ros2 run odom_plot plan_load /home/bigdavid/Nav2/src/odom_plot/localization_data/global_plan.csv

ros2 run odom_plot odom_load /home/bigdavid/Nav2/src/odom_plot/localization_data/odometry_data.csv 



