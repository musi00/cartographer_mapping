/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name    avidbots_cartographer_mapping_base.h
 * @brief   header file of cartographer mapping base class;
 * @author  Arthur Ren
 */

#ifndef AVIDBOTS_CARTOGRAPHER_MAPPING_CARTOGRAPHER_MAPPING_BASE_H
#define AVIDBOTS_CARTOGRAPHER_MAPPING_CARTOGRAPHER_MAPPING_BASE_H

/* ROS */
#include <ros/console.h>
#include <ros/ros.h>
#include "nav_msgs/GetMap.h"

/* CPP */
#include <stdio.h>
#include <string>

/* Boost */
#include <boost/filesystem.hpp>
#include "tf2/LinearMath/Matrix3x3.h"

class CartographerMappingBase {
 public:
  CartographerMappingBase();

 private:
  double occupied_threshold_;          // loaded from yaml file
  double free_threshold_;              // loaded from yaml file
  std::string output_directory_path_;  // where to put the generated map
 protected:
  ros::NodeHandle node_handle_;
  ros::Subscriber map_subscriber_;

  /*
   * @brief
  */
  void SaveMap(const nav_msgs::OccupancyGridConstPtr& map);
  virtual void MapCallback(const nav_msgs::OccupancyGrid msg) = 0;
};

#endif  // AVIDBOTS_CARTOGRAPHER_MAPPING_CARTOGRAPHER_MAPPING_BASE_H