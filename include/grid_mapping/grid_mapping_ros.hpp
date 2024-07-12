/*
 * @Author: Wei Sun 
 * @Date: 2024-07-09 09:16:07 
 * @Last Modified by: Wei Sun
 * @Last Modified time: 2024-07-12 17:43:07
 */
#ifndef SRC_GRID_MAPPING_ROS_HPP
#define SRC_GRID_MAPPING_ROS_HPP

#include <chrono>
#include <memory>
#include <string>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/srv/get_map.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h> 
// #include <message_filters/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>   

#include "tf2_ros/transform_broadcaster.h"
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

#include "grid_mapping/particle_filter.hpp"
#include "grid_mapping/pose.hpp"
#include "grid_mapping/range_reading.hpp"


// using SyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>;

using namespace std::chrono_literals;

class GridMappingRos : public rclcpp::Node{
public:
    GridMappingRos();
    ~GridMappingRos();
protected:
    void init();
    void liveGmapping(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);
    bool getOdomPose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, const rclcpp::Time& t);
    void getRangeReading(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);
    void publishMapInfo();
    void publishTransform();
    void publishLoop(double transform_publish_period);
    bool transformPose(
      const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose, 
      geometry_msgs::msg::PoseStamped & out_pose) const;

private:
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub;
  // std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>> sync_;
    typedef message_filters::sync_policies::ApproximateTime<
          nav_msgs::msg::Odometry,
          sensor_msgs::msg::LaserScan
    > SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync_;
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr mapmd_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    
    //get map
    nav_msgs::srv::GetMap::Response map_;
    double occ_thresh_;

    //param
    //GridMap
    int gm_sizeW_, gm_sizeH_;
    int gm_centerX_, gm_centerY_;
    double  gm_resolution_;
    double gm_locc_, gm_lfree_;
    int gm_thickness_;

    //Motion
    double m_a1_,m_a2_,m_a3_,m_a4_,m_sigmaX_,m_sigmaY_,m_sigmaTh_;

    //Observation
    double o_sigma_;
    double o_missLikelihood_;
    double o_n_; //modify covariance of observation
    int o_kernel_size_;
    double o_likelihood_threshold_;

    //ParticleFilter
    ParticleFilter pf_;
    int num_partices_;
    double threshold_distance_;
    double threshold_angle_;
    int threshold_hit_;

    //ScanMatcher
    double sm_translation_step_, sm_angle_step_;
    int sm_iteration_; //count of iteration
    int sm_delta_; // 10
    int sm_kernel_size_; //1
    double sm_likelihood_threshold_; //0.8
    bool sm_debug_;

    //call particle_filter
    grid_mapping::Pose gmap_initPose;
    grid_mapping::Pose gmap_odomPose;
    RangeReading rangeReading;
    Particle targetParticle;
    MatrixXd bestCells;
    geometry_msgs::msg::Pose robot_pose;

    //tf2
    std::shared_ptr<tf2_ros::TransformListener> tfL_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::string laser_frame_;
    std::string odom_frame_;
    // tf::Stamped<tf::Pose> centered_laser_pose_;
    geometry_msgs::msg::PoseStamped centered_laser_pose_;
    std::string map_frame_;
    std::string base_link_frame_;
    //发布tf2: map->odom
    double tf2_delay_;
    double transform_publish_period_;
    tf2::Transform map_to_odom_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfB_;
    std::shared_ptr<std::thread> transform_thread_;
    std::mutex map_to_odom_mutex_;

    long count;
    
    //transformPose
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_tfP_;
    double transform_tolerance_{0.1};

};

#endif //SRC_GRID_MAPPING_ROS_HPP