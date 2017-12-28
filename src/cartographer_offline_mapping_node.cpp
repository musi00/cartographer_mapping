/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name    avidbots_cartographer_offline_mapping_node.cpp
 * @brief   This node will subscribe to catographer's output and generate png and yaml files for the mapping
 * @author  Arthur Ren
 */

#include "avidbots_cartographer_mapping/cartographer_offline_mapping.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "avidbots_cartographer_offline_mapping_node");
  ROS_INFO("Starting avidbots_cartographer_offline_mapping_node...");
  CartographerOfflineMapping avidbots_cartographer_offline_mapping;
  ROS_INFO("Initialized avidbots_cartographer_offline_mapping_node...");
  ros::spin();
  return 0;
}
