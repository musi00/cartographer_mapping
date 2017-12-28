/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name    avidbots_cartographer_mapping_base.cpp
 * @brief   header file of cartographer mapping class
 * @author  Arthur Ren
 */

#include "avidbots_cartographer_mapping/cartographer_mapping_base.h"

/*
 * @name   CartographerMappingBase
 * @brief  Constructor
*/
CartographerMappingBase::CartographerMappingBase() {
  if (!node_handle_.getParam("/cartographer_mapping/save_map/occupied_threshold", occupied_threshold_)) {
    occupied_threshold_ = 0.65;
  }
  if (!node_handle_.getParam("/cartographer_mapping/save_map/free_threshold", free_threshold_)) {
    free_threshold_ = 0.195;
  }
  if (!node_handle_.getParam("/cartographer_mapping/save_map/output_directory_path", output_directory_path_)) {
    output_directory_path_ = "";
  }
}

/*
 * @name   SaveMap
 * @brief  generate the pgm and yaml file based on the input
 * @param[in] map, the OccupancyGrid from which we are generating the map
*/
void CartographerMappingBase::SaveMap(const nav_msgs::OccupancyGridConstPtr& map) {
  ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);

  // save the pgm file
  std::string mapdatafile = output_directory_path_ + "/offline_map.pgm";
  ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  FILE* out = fopen(mapdatafile.c_str(), "w");
  if (!out) {
    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    return;
  }

  fprintf(out, "P5\n# CREATOR: Cartographer_mapping_base.cpp %.3f m/pix\n%d %d\n255\n", map->info.resolution, map->info.width, map->info.height);
  for (unsigned int y = 0; y < map->info.height; y++) {
    for (unsigned int x = 0; x < map->info.width; x++) {
      unsigned int i = x + (map->info.height - y - 1) * map->info.width;
      if (map->data[i] >= occupied_threshold_ * 100) {
        fputc(0, out);
      } else if (map->data[i] <= free_threshold_ * 100 && map->data[i] >= 0) {
        fputc(254, out);
      } else {
        fputc(205, out);
      }
    }
  }
  fclose(out);

  // save the yaml file
  std::string mapmetadatafile = output_directory_path_ + "/offline_map.yaml";
  ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
  FILE* yaml = fopen(mapmetadatafile.c_str(), "w");
  geometry_msgs::Quaternion orientation = map->info.origin.orientation;
  tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: %f\nfree_thresh: %f\n\n", mapdatafile.c_str(),
          map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw, occupied_threshold_, free_threshold_);
  fclose(yaml);
  ROS_INFO("Map Save finished\n");
}