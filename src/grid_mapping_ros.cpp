#include "grid_mapping/grid_mapping_ros.hpp"

GridMappingRos::GridMappingRos():
Node("grid_mapping_ros"),
transform_thread_(nullptr)
{
    init();
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();


    odom_sub.subscribe(this, "odom", rmw_qos_profile);
    scan_sub.subscribe(this, "scan", rmw_qos_profile);
    sync_ = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>(odom_sub, scan_sub, 10);
    sync_->registerCallback(std::bind(&GridMappingRos::liveGmapping, this, std::placeholders::_1, std::placeholders::_2));
}

GridMappingRos::~GridMappingRos(){
    if(transform_thread_){
        transform_thread_->join();
    }
}

void GridMappingRos::init(){
    count = 0;
}

void GridMappingRos::liveGmapping(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) 
{
    RCLCPP_INFO(this->get_logger(),
                "I heard and synchronized the following timestamps: %u, %u",
                odom->header.stamp.sec, scan->header.stamp.sec);
}