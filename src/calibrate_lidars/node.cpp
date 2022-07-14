/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#include "calibrate_lidars/node.h"

#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>

calibrate_lidars::Node::Node(ros::NodeHandle& node_handle)
  : calibrate_lidars_ {}
  , initial_tf_ {false}
  , num_try_ {0}
  , first_ {false}
  , node_id_ {0}
{
  fixed_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle, "fixed_cloud", 10));
  var_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle, "var_cloud", 10));
  sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *fixed_sub_, *var_sub_));
  // sync_->registerCallback(boost::bind(&calibrate_lidars::Node::callback, this, _1, _2));
  output_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("output_cloud", 10);

  // cloud_sub_ = node_handle.subscribe("cloud", 1, &calibrate_lidars::Node::singleCallback, this);
  front_cloud_sub_ = node_handle.subscribe("front_cloud", 1, &calibrate_lidars::Node::frontCallback, this);
  rotation_sub_ = node_handle.subscribe("rotation", 1, &calibrate_lidars::Node::rotationCallback, this);
  correct_ = node_handle.param("correct", true);

  // matcher_.reset(new fast_gicp::FastGICP<Point, Point>());
  // matcher_->setMaxCorrespondenceDistance(1.0);
  matcher_.reset(new pclomp::NormalDistributionsTransform<Point, Point>());
  matcher_->setTransformationEpsilon(0.01);
  matcher_->setStepSize(0.1);
  matcher_->setResolution(2.0);
  matcher_->setMaximumIterations(35);
  matcher_->setOulierRatio(0.8);
  matcher_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  running_average_translation_ = Eigen::Vector3f::Zero();
  running_average_quat_ = Eigen::Quaternionf(1, 0, 0, 0);

  aggregated_.reset(new Cloud());
  yaw_angle_ = -0.05;
  rotation_data_init_ = false;
  plot_.open("plot_data.csv");
  plot_ << "x,y,z,roll,pitch,yaw\n";
}

calibrate_lidars::Node::~Node()
{
  plot_.close();
}

Eigen::Matrix4f
toMatrix(tf::StampedTransform trans)
{
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  mat(0, 3) = trans.getOrigin().x();
  mat(1, 3) = trans.getOrigin().y();
  mat(2, 3) = trans.getOrigin().z();
  Eigen::Quaterniond q;
  q.x() = trans.getRotation().x();
  q.y() = trans.getRotation().y();
  q.z() = trans.getRotation().z();
  q.w() = trans.getRotation().w();

  Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
  mat.block<3, 3>(0, 0) = rot;

  return mat.cast<float>();
}

Eigen::VectorXf
toVector(Eigen::Matrix4f mat)
{
  Eigen::VectorXf vec(6);
  vec(0) = mat(0, 3);
  vec(1) = mat(1, 3);
  vec(2) = mat(2, 3);
  Eigen::Vector3f rpy = mat.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
  vec(3) = rpy(2);
  vec(4) = rpy(1);
  vec(5) = rpy(0);
  return vec;
}

Eigen::Matrix4f
toMat(Eigen::Vector3f vec, Eigen::Quaternionf q)
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat(0, 3) = vec(0);
  mat(1, 3) = vec(1);
  mat(2, 3) = vec(2);
  Eigen::Matrix3f rot = q.normalized().toRotationMatrix();
  mat.block<3, 3>(0, 0) = rot;
  return mat;
}

Eigen::Matrix4f
toMat(Eigen::VectorXf vec)
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  for (int i = 0; i < 3; i++)
  {
    mat(i, 3) = vec(i);
  }
  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(vec(5), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(vec(4), Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(vec(3), Eigen::Vector3f::UnitX());
  mat.block<3, 3>(0, 0) = m;
  return mat;
}

Eigen::Vector3f
toVector(Eigen::Quaternionf q)
{
  Eigen::Vector3f rpy = q.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
  Eigen::Vector3f vec;
  vec(0) = rpy(2);
  vec(1) = rpy(1);
  vec(2) = rpy(0);
  return vec;
}

void
printVec(Eigen::VectorXf v)
{
  for (unsigned int i = 0; i < 6; i++)
  {
    std::cout << v(i) << " ";
  }
  std::cout << std::endl;
}

void
printVec3(Eigen::Vector3f v)
{
  for (unsigned int i = 0; i < 3; i++)
  {
    std::cout << v(i) << " ";
  }
  std::cout << std::endl;
}

bool
isFinite(Point p)
{
  return !(std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z) || std::isnan(p.x) || std::isnan(p.y)
           || std::isnan(p.z));
}

bool
isInside(Point p)
{
  return (p.x * p.x + p.y * p.y + p.z * p.z < float(1.5));
}

Cloud::Ptr
filter(Cloud::Ptr in)
{
  Cloud::Ptr filtered(new Cloud());
  for (const auto p : *in)
  {
    if (isFinite(p) && !isInside(p))
    {
      filtered->push_back(p);
    }
  }
  return filtered;
}

Cloud::Ptr
filterGround(Cloud::Ptr cloud, Eigen::Matrix4f transform)
{
  Cloud::Ptr transformed(new Cloud());
  pcl::transformPointCloud(*cloud, *transformed, transform);
  Cloud::Ptr filtered(new Cloud());
  for (const auto p : *transformed)
  {
    if (p.z > float(-0.2))
    {
      filtered->push_back(p);
    }
  }
  return filtered;
}

Cloud::Ptr
preprocessCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, Eigen::Matrix4f transform)
{
  Cloud::Ptr cloud(new Cloud());
  pcl::fromROSMsg(*msg, *cloud);
  auto filtered = filter(cloud);
  return filterGround(filtered, transform);
}

Eigen::Quaternionf
calculateAverage(Eigen::Quaternionf running, Eigen::Quaternionf data, int cur_try)
{
  Eigen::Quaternionf q((running.w() * (cur_try - 1) + data.w()) / cur_try,
                       (running.x() * (cur_try - 1) + data.x()) / cur_try,
                       (running.y() * (cur_try - 1) + data.y()) / cur_try,
                       (running.z() * (cur_try - 1) + data.z()) / cur_try);
  return q.normalized();
}

void
calibrate_lidars::Node::rotationCallback(const std_msgs::Float32 rotation)
{
  auto data = rotation.data;
  if (data > 0)
  {
    data = data * 0.8618; // from measurement 0.8263935527199463
  }

  double steering_velocity = 0;
  double dt = 0.0;
  if (!rotation_data_init_)
  {
    steering_velocity = 0;
    rotation_data_init_ = true;
    last_received_time_ = ros::Time::now();
  }
  else
  {
    dt = (ros::Time::now() - last_received_time_).toSec();
    steering_velocity = (data - old_rotation_data_) / dt;
  }
  old_rotation_data_ = data;
  double back_axis = 1.17;
  double front_axis = 0.78;

  double angular_velocity = (steering_velocity * back_axis) / (front_axis * std::cos(data) + back_axis);
  yaw_angle_ += angular_velocity * dt;
  rotation_data_ = data - yaw_angle_;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, -data);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "chassis_connector", "chassis_back"));
  // if (!initial_tf_)
  // {
  //   return;
  // }
  // Eigen::Matrix4f real_rotation = Eigen::Matrix4f::Identity();
  // real_rotation.block<3, 3>(0, 0) = Eigen::AngleAxisf(-rotation_data_, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  // Eigen::Matrix4f composite = real_rotation * initial_guess_fixed_;
  // writeVec(toVector(composite));
}

void
calibrate_lidars::Node::writeVec(Eigen::VectorXf v)
{
  for (unsigned int i = 0; i < 5; i++)
  {
    plot_ << v(i) << ",";
  }
  plot_ << v(5);
  plot_ << std::endl;
}

void
calibrate_lidars::Node::singleCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  double rotation_data = rotation_data_;
  if (!initial_tf_)
  {
    try
    {
      tf::StampedTransform initial_tf_guess;
      tf_.lookupTransform("chassis_back", cloud_msg->header.frame_id, cloud_msg->header.stamp, initial_tf_guess);
      initial_guess_fixed_ = toMatrix(initial_tf_guess);
      calibrate_lidars_.addNode(initial_guess_fixed_, 0, false);
      initial_tf_ = true;
    }
    catch (tf::TransformException& ex)
    {
      return;
    }
  }

  bool correct = false;

  auto cloud_before = preprocessCloud(cloud_msg, initial_guess_fixed_);
  Cloud::Ptr cloud(new Cloud());
  initial_guess_var_ = Eigen::Matrix4f::Identity();
  initial_guess_var_.block<3, 3>(0, 0) = Eigen::AngleAxisf(-rotation_data, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  Eigen::Matrix4f real_rotation = Eigen::Matrix4f::Identity();
  real_rotation.block<3, 3>(0, 0) = Eigen::AngleAxisf(-rotation_data, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  pcl::transformPointCloud(*cloud_before, *cloud, initial_guess_var_);
  std::cout << rotation_data << std::endl;

  if (!first_)
  {
    aggregated_.reset(new Cloud(*cloud));
    first_ = true;
    return;
  }
  to Cloud::Ptr cloud_filtered(new Cloud());
  pcl::VoxelGrid<Point> sor;
  sor.setInputCloud(aggregated_);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*cloud_filtered);
  *aggregated_ = *cloud_filtered;
  Cloud::Ptr transformed(new Cloud());
  Eigen::Matrix4f corrected;
  if (correct)
  {
    matcher_->setInputTarget(aggregated_);
    matcher_->setInputSource(cloud);
    matcher_->align(*transformed, Eigen::Matrix4f::Identity());
    corrected = matcher_->getFinalTransformation();
    pcl::transformPointCloud(*cloud, *transformed, corrected);
  }
  else
  {
    *transformed = *cloud;
  }

  *aggregated_ += *transformed;
  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*aggregated_, out);
  out.header = cloud_msg->header;
  out.header.frame_id = "chassis_back";
  output_pub_.publish(out);
  node_id_++;

  if (correct)
  {
    Eigen::Matrix4f composite = corrected * initial_guess_var_ * initial_guess_fixed_;
    writeVec(toVector(composite));
    printVec(toVector(composite));

    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);
    information.topLeftCorner(3, 3).array() /= 0.5;
    information.bottomRightCorner(3, 3).array() /= 0.1;

    calibrate_lidars_.addNode(composite, node_id_, true);
    calibrate_lidars_.addEdge(0, node_id_, real_rotation, information);
    if (node_id_ % 10 == 0)
    {
      calibrate_lidars_.optimize();
      auto data = calibrate_lidars_.retrieveCorrected();
      std::cout << "=============OPTIMIZE===========\n";
      printVec(toVector(data[0]));
      std::cout << "===============END==============\n";
    }
  }
}

void
calibrate_lidars::Node::frontCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  double rotation_data = yaw_angle_;
  if (!initial_tf_)
  {
    try
    {
      tf::StampedTransform initial_tf_guess;
      tf_.lookupTransform("chassis_connector", cloud_msg->header.frame_id, cloud_msg->header.stamp, initial_tf_guess);
      initial_guess_fixed_ = toMatrix(initial_tf_guess);
      calibrate_lidars_.addNode(initial_guess_fixed_, 0, false);
      initial_tf_ = true;
    }
    catch (tf::TransformException& ex)
    {
      return;
    }
  }

  bool correct = correct_;

  auto cloud_before = preprocessCloud(cloud_msg, initial_guess_fixed_);
  Cloud::Ptr cloud(new Cloud());
  initial_guess_var_ = Eigen::Matrix4f::Identity();
  initial_guess_var_.block<3, 3>(0, 0) = Eigen::AngleAxisf(rotation_data, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  Eigen::Matrix4f real_rotation = Eigen::Matrix4f::Identity();
  real_rotation.block<3, 3>(0, 0) = Eigen::AngleAxisf(rotation_data, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  pcl::transformPointCloud(*cloud_before, *cloud, initial_guess_var_);
  std::cout << rotation_data << std::endl;

  if (!first_)
  {
    aggregated_.reset(new Cloud(*cloud));
    first_ = true;
    return;
  }
  Cloud::Ptr cloud_filtered(new Cloud());
  pcl::VoxelGrid<Point> sor;
  sor.setInputCloud(aggregated_);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*cloud_filtered);
  *aggregated_ = *cloud_filtered;
  Cloud::Ptr transformed(new Cloud());
  Eigen::Matrix4f corrected;
  if (correct)
  {
    matcher_->setInputTarget(aggregated_);
    matcher_->setInputSource(cloud);
    matcher_->align(*transformed, Eigen::Matrix4f::Identity());
    corrected = matcher_->getFinalTransformation();
    pcl::transformPointCloud(*cloud, *transformed, corrected);
  }
  else
  {
    *transformed = *cloud;
  }

  *aggregated_ += *transformed;
  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*aggregated_, out);
  out.header = cloud_msg->header;
  out.header.frame_id = "chassis_connector";
  output_pub_.publish(out);
  node_id_++;

  if (correct)
  {
    std::cout << "****************\n";
    printVec(toVector(initial_guess_var_));
    auto actual = corrected * initial_guess_var_;
    printVec(toVector(actual));
    Eigen::Matrix4f composite = corrected * initial_guess_var_ * initial_guess_fixed_;
    writeVec(toVector(composite));
    printVec(toVector(composite));

    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);
    information.topLeftCorner(3, 3).array() /= 0.5;
    information.bottomRightCorner(3, 3).array() /= 0.1;

    calibrate_lidars_.addNode(composite, node_id_, true);
    calibrate_lidars_.addEdge(0, node_id_, real_rotation, information);
    if (node_id_ % 10 == 0)
    {
      calibrate_lidars_.optimize();
      auto data = calibrate_lidars_.retrieveCorrected();
      std::cout << "=============OPTIMIZE===========\n";
      printVec(toVector(data[0]));
      std::cout << "===============END==============\n";
    }
  }
}

void
calibrate_lidars::Node::callback(const sensor_msgs::PointCloud2::ConstPtr& fixed,
                                 const sensor_msgs::PointCloud2::ConstPtr& var)
{
  if (!initial_tf_)
  {
    try
    {
      tf::StampedTransform initial_tf_guess;
      tf_.lookupTransform("chassis_back", fixed->header.frame_id, fixed->header.stamp, initial_tf_guess);
      initial_guess_fixed_ = toMatrix(initial_tf_guess);
      tf_.lookupTransform("chassis_back", var->header.frame_id, var->header.stamp, initial_tf_guess);
      initial_guess_var_ = toMatrix(initial_tf_guess);
      initial_tf_ = true;
      calibrate_lidars_.addNode(initial_guess_fixed_, 0, true);
      calibrate_lidars_.addNode(initial_guess_var_, 1, false);
    }
    catch (tf::TransformException& ex)
    {
      return;
    }
  }

  auto fixed_filter = preprocessCloud(fixed, initial_guess_fixed_);
  auto var_filter = preprocessCloud(var, initial_guess_var_);

  matcher_->setInputTarget(fixed_filter);
  matcher_->setInputSource(var_filter);

  Cloud::Ptr transformed(new Cloud());
  matcher_->align(*transformed, Eigen::Matrix4f::Identity());
  Eigen::Matrix4f corrected = matcher_->getFinalTransformation();

  pcl::transformPointCloud(*var_filter, *transformed, corrected);
  for (auto& p : *transformed)
  {
    p.intensity = 100;
  }
  for (auto& p : *fixed_filter)
  {
    p.intensity = 255;
  }
  *transformed += *fixed_filter;

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*transformed, out);
  out.header = fixed->header;
  out.header.frame_id = "chassis_back";
  output_pub_.publish(out);
  std::cout << matcher_->getFitnessScore() << std::endl;
  // printVec(toVector(corrected));
  // Eigen::Matrix4f var_corrected = corrected * initial_guess_var_;
  // printVec(toVector(var_corrected));
  if (fabs(corrected(0, 3)) < 0.2 && fabs(corrected(1, 3)) < 0.2 && fabs(corrected(2, 3)) < 0.2
      && matcher_->getFitnessScore() < 10.0)
  {
    num_try_ += 1;
    running_average_translation_ = (running_average_translation_ * (num_try_ - 1)
                                    + Eigen::Vector3f(corrected(0, 3), corrected(1, 3), corrected(2, 3)))
                                   / num_try_;
    printVec3(running_average_translation_);
    Eigen::Quaternionf curq(corrected.block<3, 3>(0, 0));
    running_average_quat_ = calculateAverage(running_average_quat_, curq, num_try_);
    printVec(toVector(toMat(running_average_translation_, running_average_quat_) * initial_guess_var_));
    // calibrate_lidars_.addEdge(1, 0, corrected, Eigen::MatrixXd::Identity(6, 6));
  }
  // initial_guess_var_ = var_corrected;
}
