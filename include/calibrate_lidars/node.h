/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef CALIBRATE_LIDARS_NODE_H_
#define CALIBRATE_LIDARS_NODE_H_

#include "calibrate_lidars/calibrate_lidars.h"

#include <Eigen/Core>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fstream>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Cloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
    MySyncPolicy;

namespace calibrate_lidars
{
class Node
{
public:
  Node(ros::NodeHandle& node_handle);
  ~Node();

private:
  CalibrateLidars calibrate_lidars_;
  ros::Publisher output_pub_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> fixed_sub_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> var_sub_;
  boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  tf::TransformListener tf_;
  // std::shared_ptr<fast_gicp::FastGICP<Point, Point>> matcher_;
  std::shared_ptr<pclomp::NormalDistributionsTransform<Point, Point>> matcher_;
  std::ofstream plot_;
  void writeVec(Eigen::VectorXf v);

  Cloud::Ptr aggregated_;
  ros::Subscriber front_cloud_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber rotation_sub_;
  bool first_;
  bool correct_;

  ros::Time last_received_time_;
  bool rotation_data_init_;
  double old_rotation_data_;
  double yaw_angle_;

  double rotation_data_;
  int node_id_;

  void singleCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void frontCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void rotationCallback(const std_msgs::Float32 rotation);

  void callback(const sensor_msgs::PointCloud2::ConstPtr& fixed, const sensor_msgs::PointCloud2::ConstPtr& var);
  bool initial_tf_;
  Eigen::Matrix4f initial_guess_fixed_;
  Eigen::Matrix4f initial_guess_var_;
  Eigen::Vector3f running_average_translation_;
  Eigen::Quaternionf running_average_quat_;
  int num_try_;
};

} // namespace calibrate_lidars

#endif
