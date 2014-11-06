/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file velodyne_post_node.cpp
    \brief This file is the ROS node for post-processing Velodyne data.
  */

#include <ros/ros.h>

#include "VelodynePostNode.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "velodyne_post");
  ros::NodeHandle nh("~");
  try {
    velodyne::VelodynePostNode vn(nh);
    while (ros::ok()) {
      vn.updatePointCloudSubscription();
      vn.sleep();
      ros::spinOnce();
    }
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }
  return 0;
}
