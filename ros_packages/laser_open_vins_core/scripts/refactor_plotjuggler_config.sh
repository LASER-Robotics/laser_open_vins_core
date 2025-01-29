#!/bin/bash

PACKAGE_PATH=$(ros2 pkg prefix --share laser_open_vins_core)

cp $PACKAGE_PATH/plotjuggler/imu_layout.xml /tmp/imu_layout.xml

sed -i "s/uav[0-9]/$UAV_NAME/g" /tmp/imu_layout.xml
