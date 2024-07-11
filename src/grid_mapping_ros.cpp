/*
 * @Author: Wei Sun 
 * @Date: 2024-07-09 09:15:43 
 * @Last Modified by: Wei Sun
 * @Last Modified time: 2024-07-11 13:54:55
 */
#include "grid_mapping/grid_mapping_ros.hpp"

GridMappingRos::GridMappingRos():
Node("grid_mapping_ros"),
sync_(SyncPolicy(10), odom_sub, scan_sub),
transform_thread_(nullptr)
{
    init();
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();


    // odom_sub.subscribe(this, "odom", rmw_qos_profile);
    // scan_sub.subscribe(this, "scan", rmw_qos_profile);
    // sync_ = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>(odom_sub, scan_sub, 10);
    // sync_->registerCallback(std::bind(&GridMappingRos::liveGmapping, this, std::placeholders::_1, std::placeholders::_2));

    // auto odom_sub = std::make_unique<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "/odom");
    // auto scan_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, "/scan");
    // auto sync = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), *odom_sub, *scan_sub);
    // sync->registerCallback(std::bind(&GridMappingRos::liveGmapping, this, std::placeholders::_1, std::placeholders::_2));
    odom_sub.subscribe(this, "/odom");
    scan_sub.subscribe(this, "/scan");
    sync_.registerCallback(&GridMappingRos::liveGmapping, this);

}

GridMappingRos::~GridMappingRos(){
    if(transform_thread_){
        transform_thread_->join();
    }
}

void GridMappingRos::init(){
    count = 0;
    RCLCPP_INFO(this->get_logger(), "grid mapping start...");
}

void GridMappingRos::liveGmapping(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) 
{
    RCLCPP_INFO(this->get_logger(),
                "I heard and synchronized the following timestamps: %u, %u",
                odom->header.stamp.sec, scan->header.stamp.sec);
}