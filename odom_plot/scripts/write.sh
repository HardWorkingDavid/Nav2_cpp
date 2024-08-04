#!/bin/bash

echo "write_to_csv(odom/plan):start!!!"

cd /home/bigdavid/Nav2

source install/setup.bash

# 每次写入清除csv文件里的内容
echo -n > /home/bigdavid/Nav2/src/odom_plot/localization_data/global_plan.csv
echo -n > /home/bigdavid/Nav2/src/odom_plot/localization_data/odometry_data.csv

echo "record odom plan..."
ros2 run odom_plot odom_write & ros2 run odom_plot plan_write

