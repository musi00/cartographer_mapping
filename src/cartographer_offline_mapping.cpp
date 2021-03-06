/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name    avidbots_cartographer_offline_mapping.cpp
 * @brief   header file of cartographer mapping class
 * @author  Arthur Ren
 */

#include "avidbots_cartographer_mapping/cartographer_offline_mapping.h"

/*
 * @name   CartographerOfflineMapping
 * @brief  constructor
*/

CartographerOfflineMapping::CartographerOfflineMapping() : CartographerMappingBase() {
  map_subscriber_ = node_handle_.subscribe("/map", 1, &CartographerOfflineMapping::MapCallback, this);
  timestamp_sec_ = 0;
}

/*
 * @name   MapCallback
 * @brief  Call back function for topic /map
 * @param[in] msg, the msg received from topic /map
*/
void CartographerOfflineMapping::MapCallback(const nav_msgs::OccupancyGrid msg) {
  // in order to make sure the msg we received is the fianal map generated by
  // offline node, we first check if the offline node is still running

  // record the time stamp of the map
  if (!(check_file_ = popen("rosnode list | grep offline_node", "r"))) {
    ROS_FATAL("Cannnot run \"rosnode list\" command!");
  }
  // if we didn't get anything, then the node have finished
  if (fgets(buff_, sizeof(buff_), check_file_) == NULL) {
    // if the msg's timestamp didn't change over two msgs, we know the map processing is finished
    if (msg.header.stamp.sec == timestamp_sec_) {
      // convert the message to a shared_ptr
      const boost::shared_ptr<nav_msgs::OccupancyGrid> map = boost::make_shared<nav_msgs::OccupancyGrid>(msg);
      SaveMap(map);
      pclose(check_file_);
      system("rosnode kill /cartographer_occupancy_grid_node /rviz");
      ros::shutdown();
    } else {
      timestamp_sec_ = msg.header.stamp.sec;
    }
  }
}