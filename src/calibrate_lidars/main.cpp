/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#include "calibrate_lidars/node.h"

#include <ros/console.h>
#include <ros/node_handle.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_lidars");

  ros::NodeHandle node_handle("~");
  calibrate_lidars::Node calibrate_lidars_node(node_handle);

  ROS_INFO("Initialized");
  ros::spin();
}
