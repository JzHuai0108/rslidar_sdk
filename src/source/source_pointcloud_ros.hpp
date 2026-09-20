/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include "source/source.hpp"

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#ifdef ENABLE_IMU_DATA_PARSE
  #include "sensor_msgs/Imu.h"
#endif
namespace robosense
{
namespace lidar
{

inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointCloudMsg& rs_msg, const std::string& frame_id, bool send_by_rows)
{
  sensor_msgs::PointCloud2 ros_msg;

  int fields = 4;
#ifdef POINT_TYPE_XYZIF
  fields = 5;
#elif defined(POINT_TYPE_XYZIRT)
  fields = 6;
#elif defined(POINT_TYPE_XYZIRTF)
  fields = 7;
#endif
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

  if (send_by_rows)
  {
    ros_msg.width = rs_msg.width; 
    ros_msg.height = rs_msg.height; 
  }
  else
  {
    ros_msg.width = rs_msg.height; // exchange width and height to be compatible with pcl::PointCloud<>
    ros_msg.height = rs_msg.width; 
  }

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF) 
  offset = addPointField(ros_msg, "feature", 1, sensor_msgs::PointField::UINT8, offset);
#endif

#if 0
  std::cout << "off:" << offset << std::endl;
#endif

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = rs_msg.is_dense;
  ros_msg.data.resize(ros_msg.point_step * ros_msg.width * ros_msg.height);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");

#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF) 
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_feature_(ros_msg, "feature");
#endif

  if (send_by_rows)
  {
    for (size_t i = 0; i < rs_msg.height; i++)
    {
      for (size_t j = 0; j < rs_msg.width; j++)
      {
        const LidarPointCloudMsg::PointT& point = rs_msg.points[i + j * rs_msg.height];

        *iter_x_ = point.x;
        *iter_y_ = point.y;
        *iter_z_ = point.z;
        *iter_intensity_ = point.intensity;

        ++iter_x_;
        ++iter_y_;
        ++iter_z_;
        ++iter_intensity_;

#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
        *iter_ring_ = point.ring;
        *iter_timestamp_ = point.timestamp;

        ++iter_ring_;
        ++iter_timestamp_;
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF) 
        *iter_feature_ = point.feature;
        ++iter_feature_;
#endif
        
      }
    }
  }
  else
  {
    for (size_t i = 0; i < rs_msg.points.size(); i++)
    {
      const LidarPointCloudMsg::PointT& point = rs_msg.points[i];

      *iter_x_ = point.x;
      *iter_y_ = point.y;
      *iter_z_ = point.z;
      *iter_intensity_ = point.intensity;

      ++iter_x_;
      ++iter_y_;
      ++iter_z_;
      ++iter_intensity_;

#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
      *iter_ring_ = point.ring;
      *iter_timestamp_ = point.timestamp;

      ++iter_ring_;
      ++iter_timestamp_;
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF) 
        *iter_feature_ = point.feature;
        ++iter_feature_;
#endif
    }
  }

  ros_msg.header.seq = rs_msg.seq;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}
#ifdef ENABLE_IMU_DATA_PARSE
sensor_msgs::Imu toRosMsg(const std::shared_ptr<ImuData>& data, const std::string& frame_id)
{
  sensor_msgs::Imu imu_msg;

  imu_msg.header.stamp = imu_msg.header.stamp.fromSec(data->timestamp);
  imu_msg.header.frame_id = frame_id;
  // Set IMU data
  imu_msg.angular_velocity.x = data->angular_velocity_x;
  imu_msg.angular_velocity.y = data->angular_velocity_y;
  imu_msg.angular_velocity.z = data->angular_velocity_z;

  imu_msg.linear_acceleration.x = data->linear_acceleration_x;
  imu_msg.linear_acceleration.y = data->linear_acceleration_y;
  imu_msg.linear_acceleration.z = data->linear_acceleration_z;
  return imu_msg;
}
#endif
class DestinationPointCloudRos : public DestinationPointCloud
{
public:

  virtual void init(const YAML::Node& config);
  virtual void sendPointCloud(const LidarPointCloudMsg& msg);
  virtual ~DestinationPointCloudRos() = default;
#ifdef ENABLE_IMU_DATA_PARSE
  virtual void sendImuData(const std::shared_ptr<ImuData> & data);
#endif
private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher pub_; 
#ifdef ENABLE_IMU_DATA_PARSE
  ros::Publisher imu_pub_; 
#endif
  std::string frame_id_;
  bool send_by_rows_;
};

inline void DestinationPointCloudRos::init(const YAML::Node& config)
{
  yamlRead<bool>(config["ros"], 
      "ros_send_by_rows", send_by_rows_, false);

  bool dense_points;
  yamlRead<bool>(config["driver"], "dense_points", dense_points, false);
  if (dense_points)
    send_by_rows_ = false;

  yamlRead<std::string>(config["ros"], 
      "ros_frame_id", frame_id_, "rslidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], 
      "ros_send_point_cloud_topic", ros_send_topic, "rslidar_points");



  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
#ifdef ENABLE_IMU_DATA_PARSE
  std::string ros_send_imu_data_topic;
  yamlRead<std::string>(config["ros"], 
      "ros_send_imu_data_topic", ros_send_imu_data_topic, "rslidar_imu_data");
  imu_pub_ = nh_->advertise<sensor_msgs::Imu>(ros_send_imu_data_topic, 1000);
#endif
}

inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg& msg)
{
  pub_.publish(toRosMsg(msg, frame_id_, send_by_rows_));
}
#ifdef ENABLE_IMU_DATA_PARSE
inline void DestinationPointCloudRos::sendImuData(const std::shared_ptr<ImuData> & data)
{
  imu_pub_.publish(toRosMsg(data, frame_id_));
}
#endif
}  // namespace lidar
}  // namespace robosense

#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#ifdef ENABLE_IMU_DATA_PARSE
  #include <sensor_msgs/msg/imu.hpp>
#endif
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <utility>

namespace robosense
{
namespace lidar
{

inline sensor_msgs::msg::PointCloud2 toRosMsg(const LidarPointCloudMsg& rs_msg, const std::string& frame_id, bool send_by_rows)
{
  sensor_msgs::msg::PointCloud2 ros_msg;

  int fields = 4;
#ifdef POINT_TYPE_XYZIF
  fields = 5;
#elif defined(POINT_TYPE_XYZIRT)
  fields = 6;
#elif defined(POINT_TYPE_XYZIRTF)
  fields = 7;
#endif
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

  if (send_by_rows)
  {
    ros_msg.width = rs_msg.width; 
    ros_msg.height = rs_msg.height; 
  }
  else
  {
    ros_msg.width = rs_msg.height; // exchange width and height to be compatible with pcl::PointCloud<>
    ros_msg.height = rs_msg.width; 
  }

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);

#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF) 
  offset = addPointField(ros_msg, "feature", 1, sensor_msgs::msg::PointField::UINT8, offset);
#endif

#if 0
  std::cout << "off:" << offset << std::endl;
#endif

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = rs_msg.is_dense;
  ros_msg.data.resize(ros_msg.point_step * ros_msg.width * ros_msg.height);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF) 
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_feature_(ros_msg, "feature");
#endif

  if (send_by_rows)
  {
    for (size_t i = 0; i < rs_msg.height; i++)
    {
      for (size_t j = 0; j < rs_msg.width; j++)
      {
        const LidarPointCloudMsg::PointT& point = rs_msg.points[i + j * rs_msg.height];

        *iter_x_ = point.x;
        *iter_y_ = point.y;
        *iter_z_ = point.z;
        *iter_intensity_ = point.intensity;

        ++iter_x_;
        ++iter_y_;
        ++iter_z_;
        ++iter_intensity_;

#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
      *iter_ring_ = point.ring;
      *iter_timestamp_ = point.timestamp;

      ++iter_ring_;
      ++iter_timestamp_;
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF) 
        *iter_feature_ = point.feature;
        ++iter_feature_;
#endif

      }
    }
  }
  else
  {
    for (size_t i = 0; i < rs_msg.points.size(); i++)
    {
      const LidarPointCloudMsg::PointT& point = rs_msg.points[i];

      *iter_x_ = point.x;
      *iter_y_ = point.y;
      *iter_z_ = point.z;
      *iter_intensity_ = point.intensity;

      ++iter_x_;
      ++iter_y_;
      ++iter_z_;
      ++iter_intensity_;

#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
      *iter_ring_ = point.ring;
      *iter_timestamp_ = point.timestamp;

      ++iter_ring_;
      ++iter_timestamp_;
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF) 
      *iter_feature_ = point.feature;
      ++iter_feature_;
#endif
    }
  }

  ros_msg.header.stamp.sec = (uint32_t)floor(rs_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((rs_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}

// Convert the camera-style Airy mounting axes (right, down, forward) to the
// ROS mobile-robot convention (forward, left, up), then keep only the height
// slice useful for teleoperation. The original cloud remains untouched.
inline sensor_msgs::msg::PointCloud2 toTeleopRosMsg(
    const sensor_msgs::msg::PointCloud2& source,
    const std::string& frame_id,
    float min_z,
    float max_z)
{
  sensor_msgs::msg::PointCloud2 output = source;
  output.header.frame_id = frame_id;
  output.height = 1;
  output.width = 0;
  output.row_step = 0;
  output.is_dense = true;
  output.data.clear();
  output.data.reserve(source.data.size());

  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t z_offset = 0;
  bool have_x = false;
  bool have_y = false;
  bool have_z = false;
  for (const auto& field : source.fields)
  {
    if (field.name == "x")
    {
      x_offset = field.offset;
      have_x = true;
    }
    else if (field.name == "y")
    {
      y_offset = field.offset;
      have_y = true;
    }
    else if (field.name == "z")
    {
      z_offset = field.offset;
      have_z = true;
    }
  }
  if (!have_x || !have_y || !have_z || source.point_step == 0)
  {
    return output;
  }

  const size_t point_count = source.data.size() / source.point_step;
  for (size_t index = 0; index < point_count; ++index)
  {
    const uint8_t* input = source.data.data() + index * source.point_step;
    float old_x;
    float old_y;
    float old_z;
    std::memcpy(&old_x, input + x_offset, sizeof(float));
    std::memcpy(&old_y, input + y_offset, sizeof(float));
    std::memcpy(&old_z, input + z_offset, sizeof(float));

    const float new_x = old_z;
    const float new_y = -old_x;
    const float new_z = -old_y;
    if (!std::isfinite(new_x) || !std::isfinite(new_y) ||
        !std::isfinite(new_z) || new_z < min_z || new_z > max_z)
    {
      continue;
    }

    const size_t output_offset = output.data.size();
    output.data.resize(output_offset + source.point_step);
    uint8_t* destination = output.data.data() + output_offset;
    std::memcpy(destination, input, source.point_step);
    std::memcpy(destination + x_offset, &new_x, sizeof(float));
    std::memcpy(destination + y_offset, &new_y, sizeof(float));
    std::memcpy(destination + z_offset, &new_z, sizeof(float));
    ++output.width;
  }

  output.row_step = output.width * output.point_step;
  return output;
}
#ifdef ENABLE_IMU_DATA_PARSE
sensor_msgs::msg::Imu toRosMsg(const std::shared_ptr<ImuData>& data, const std::string& frame_id)
{
  sensor_msgs::msg::Imu imu_msg;

  imu_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(data->timestamp * 1e9));
  imu_msg.header.frame_id = frame_id;
  // Set IMU data
  imu_msg.angular_velocity.x = data->angular_velocity_x;
  imu_msg.angular_velocity.y = data->angular_velocity_y;
  imu_msg.angular_velocity.z = data->angular_velocity_z;

  imu_msg.linear_acceleration.x = data->linear_acceleration_x;
  imu_msg.linear_acceleration.y = data->linear_acceleration_y;
  imu_msg.linear_acceleration.z = data->linear_acceleration_z;
  return imu_msg;
}
#endif
class DestinationPointCloudRos : virtual public DestinationPointCloud
{
public:

  virtual void init(const YAML::Node& config);
  virtual void sendPointCloud(const LidarPointCloudMsg& msg);
  virtual void sendTemperature(float temperature);
#ifdef ENABLE_IMU_DATA_PARSE
  virtual void sendImuData(const std::shared_ptr<ImuData> & data);
#endif
  virtual ~DestinationPointCloudRos() = default;

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr teleop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
#ifdef ENABLE_IMU_DATA_PARSE
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
#endif
  std::string frame_id_;
  std::string teleop_frame_id_;
  float teleop_min_z_;
  float teleop_max_z_;
  bool send_by_rows_;
};

inline void DestinationPointCloudRos::init(const YAML::Node& config)
{
  yamlRead<bool>(config["ros"], 
      "ros_send_by_rows", send_by_rows_, false);

  bool dense_points;
  yamlRead<bool>(config["driver"], "dense_points", dense_points, false);
  if (dense_points)
    send_by_rows_ = false;

  yamlRead<std::string>(config["ros"], 
      "ros_frame_id", frame_id_, "rslidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], 
      "ros_send_point_cloud_topic", ros_send_topic, "rslidar_points");

  size_t ros_queue_length;
  yamlRead<size_t>(config["ros"], "ros_queue_length", ros_queue_length, 100);

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "rslidar_points_destination_" << node_index++;

  node_ptr_.reset(new rclcpp::Node(node_name.str()));

  pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_send_topic, ros_queue_length);

  bool send_teleop_point_cloud;
  yamlRead<bool>(config["ros"],
      "ros_send_teleop_point_cloud", send_teleop_point_cloud, false);
  if (send_teleop_point_cloud)
  {
    std::string teleop_topic;
    yamlRead<std::string>(config["ros"],
        "ros_send_teleop_point_cloud_topic", teleop_topic, "rslidar_points_teleop");
    yamlRead<std::string>(config["ros"],
        "ros_teleop_frame_id", teleop_frame_id_, "rslidar_teleop");
    yamlRead<float>(config["ros"], "ros_teleop_min_z", teleop_min_z_, -0.25f);
    yamlRead<float>(config["ros"], "ros_teleop_max_z", teleop_max_z_, 0.50f);
    if (teleop_min_z_ > teleop_max_z_)
    {
      std::swap(teleop_min_z_, teleop_max_z_);
    }
    auto teleop_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    teleop_qos.best_effort();
    teleop_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
        teleop_topic, teleop_qos);
  }

  std::string ros_send_temperature_topic;
  yamlRead<std::string>(config["ros"],
      "ros_send_temperature_topic", ros_send_temperature_topic, "rslidar_temperature");
  temperature_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Temperature>(
      ros_send_temperature_topic, rclcpp::SensorDataQoS());

#ifdef ENABLE_IMU_DATA_PARSE
  std::string ros_send_imu_data_topic;
  yamlRead<std::string>(config["ros"], 
      "ros_send_imu_data_topic", ros_send_imu_data_topic, "rslidar_imu_data");
  imu_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(ros_send_imu_data_topic, 1000);
#endif

}

inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg& msg)
{
  auto ros_msg = toRosMsg(msg, frame_id_, send_by_rows_);
  if (teleop_pub_ && teleop_pub_->get_subscription_count() > 0)
  {
    auto teleop_msg = toTeleopRosMsg(
        ros_msg, teleop_frame_id_, teleop_min_z_, teleop_max_z_);
    pub_->publish(std::move(ros_msg));
    teleop_pub_->publish(std::move(teleop_msg));
  }
  else
  {
    pub_->publish(std::move(ros_msg));
  }
}

inline void DestinationPointCloudRos::sendTemperature(float temperature)
{
  sensor_msgs::msg::Temperature msg;
  msg.header.stamp = node_ptr_->get_clock()->now();
  msg.header.frame_id = frame_id_;
  msg.temperature = temperature;
  msg.variance = 0.0;
  temperature_pub_->publish(msg);
}
#ifdef ENABLE_IMU_DATA_PARSE
inline void DestinationPointCloudRos::sendImuData(const std::shared_ptr<ImuData> & data)
{
  imu_pub_->publish(toRosMsg(data, frame_id_));
}
#endif
}  // namespace lidar
}  // namespace robosense

#endif
