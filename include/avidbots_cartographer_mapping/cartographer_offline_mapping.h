/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name    avidbots_cartographer_offline_mapping.h
 * @brief   header file of cartographer's offline mapping class
 * @author  Arthur Ren
 */

#ifndef AVIDBOTS_CARTOGRAPHER_MAPPING_CARTOGRAPHER_OFFLINE_MAPPING_H
#define AVIDBOTS_CARTOGRAPHER_MAPPING_CARTOGRAPHER_OFFLINE_MAPPING_H

/* ROS */
#include <ros/console.h>
#include <ros/ros.h>

/* CPP */
#include <stdio.h>
#include <string>

/* Boost */
#include <boost/filesystem.hpp>
#include "tf2/LinearMath/Matrix3x3.h"

/* Local */
#include "avidbots_cartographer_mapping/cartographer_mapping_base.h"
class CartographerOfflineMapping : public CartographerMappingBase {
 public:
  CartographerOfflineMapping();

 private:
  FILE *check_file_;   // used to buffer popen's output
  char buff_[30];      // used to fgets check_file_'s output
  int timestamp_sec_;  // used to check if the map has finished processing
  void MapCallback(const nav_msgs::OccupancyGrid msg);
};

#endif  // AVIDBOTS_CARTOGRAPHER_MAPPING_CARTOGRAPHER_OFFLINE_MAPPING_H