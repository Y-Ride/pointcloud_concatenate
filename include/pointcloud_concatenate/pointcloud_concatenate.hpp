#pragma once           // Only include once per compile
#ifndef POINTCLOUD_CONCATENATE  // Conditional compiling
#define POINTCLOUD_CONCATENATE

// Includes
#include <ros/ros.h>  // ROS header

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>

// Macro to warn about unset parameters
#define ROSPARAM_WARN(param_name, default_val)                               \
  std::cout << "\033[33m"                                                    \
            << "[WARN] Param is not set: " << param_name                          \
            << ". Setting to default value: " << default_val << "\033[0m\n"  \
            << std::endl

// Define class
class PointcloudConcatenate {
public:
  // Constructor and destructor
  PointcloudConcatenate(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~PointcloudConcatenate();

  // Public functions
  void handleParams();
  void update();
  double getHz();

  // Public variables and objects
  
private:
  // Private functions
  void subCallbackCloudFront(sensor_msgs::PointCloud2 msg);
  void subCallbackCloudBack(sensor_msgs::PointCloud2 msg);
  void subCallbackCloudLeft(sensor_msgs::PointCloud2 msg);
  void subCallbackCloudRight(sensor_msgs::PointCloud2 msg);
  void subCallbackCloudTop(sensor_msgs::PointCloud2 msg);
  void publishPointcloud(sensor_msgs::PointCloud2 cloud);

  // Private variables and objects
  ros::NodeHandle nh_;
  std::string node_name_;

  // Parameters
  std::string param_frame_target_;
  int param_clouds_;
  double param_hz_;

  // Publisher and subscribers
  ros::Subscriber sub_cloud_front = nh_.subscribe("cloud_front", 1, &PointcloudConcatenate::subCallbackCloudFront, this);
  ros::Subscriber sub_cloud_back = nh_.subscribe("cloud_back", 1, &PointcloudConcatenate::subCallbackCloudBack, this);
  ros::Subscriber sub_cloud_left = nh_.subscribe("cloud_left", 1, &PointcloudConcatenate::subCallbackCloudLeft, this);
  ros::Subscriber sub_cloud_right = nh_.subscribe("cloud_right", 1, &PointcloudConcatenate::subCallbackCloudRight, this);
  ros::Subscriber sub_cloud_top = nh_.subscribe("cloud_top", 1, &PointcloudConcatenate::subCallbackCloudTop, this);
  ros::Publisher pub_cloud_out = nh_.advertise<sensor_msgs::PointCloud2>(node_name_ + "/cloud_out", 1);

  // Other

  sensor_msgs::PointCloud2 cloud_front;
  sensor_msgs::PointCloud2 cloud_back;
  sensor_msgs::PointCloud2 cloud_left;
  sensor_msgs::PointCloud2 cloud_right;
  sensor_msgs::PointCloud2 cloud_top;
  sensor_msgs::PointCloud2 cloud_out;
  bool cloud_front_received = false;
  bool cloud_back_received = false;
  bool cloud_left_received = false;
  bool cloud_right_received = false;
  bool cloud_top_received = false;
  bool cloud_front_received_recent = false;
  bool cloud_back_received_recent = false;
  bool cloud_left_received_recent = false;
  bool cloud_right_received_recent = false;
  bool cloud_top_received_recent = false;

  // Initialization tf2 listener
  boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
  boost::shared_ptr<tf2_ros::TransformListener> tfListener;
};

#endif