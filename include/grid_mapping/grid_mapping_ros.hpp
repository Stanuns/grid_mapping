#ifndef SRC_GRID_MAPPING_ROS_HPP
#define SRC_GRID_MAPPING_ROS_HPP

#include <chrono>
#include <memory>
#include <string>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>    

#include "geometry_msgs/msg/point_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#ifdef TF2_CPP_HEADERS
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
  #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif


using namespace std::chrono_literals;

class GridMappingRos : public rclcpp::Node{
public:
    GridMappingRos();
    ~GridMappingRos();
protected:
    void init();
    void liveGmapping(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);

private:
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub;
  std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>> sync_;

  long count;
  std::shared_ptr<std::thread> transform_thread_;

};

#endif //SRC_GRID_MAPPING_ROS_HPP