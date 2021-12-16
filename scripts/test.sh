#!/bin/bash
roslaunch dataset_pub pub.launch &
rosrun image_view image_view image:=/left/image_rect_color

