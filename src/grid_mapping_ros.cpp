/*
 * @Author: Wei Sun 
 * @Date: 2024-07-09 09:15:43 
 * @Last Modified by: Wei Sun
 * @Last Modified time: 2024-07-15 20:40:46
 */
#include "rclcpp/rclcpp.hpp"
#include "grid_mapping/grid_mapping_ros.hpp"
#include "grid_mapping/grid_map.hpp"
#include "grid_mapping/motion.hpp"
#include "grid_mapping/observation.hpp"
#include "grid_mapping/scan_matcher.hpp"
#include "grid_mapping/particle_filter.hpp"

#include "tf2/LinearMath/Quaternion.h"

#define MAP_IDX(w,h, i, j) ((w)*((h)-(i)-1)+(j))

GridMappingRos::GridMappingRos():
Node("grid_mapping_ros"),
sync_(SyncPolicy(10), odom_sub, scan_sub),
transform_thread_(nullptr)
{
    init();
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tfL_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    map_to_odom_.setIdentity();

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

    transform_thread_ = std::make_shared<std::thread>
            (std::bind(&GridMappingRos::publishLoop, this, transform_publish_period_));
   
}

GridMappingRos::~GridMappingRos(){
    if(transform_thread_){
        transform_thread_->join();
    }
}

void GridMappingRos::init(){
    count = 0;
    RCLCPP_INFO(this->get_logger(), "grid mapping start...");
    //GridMap
    gm_sizeW_ = 50; //3500个grid cell
    gm_sizeH_ = 50;
    gm_centerX_ = (int)(gm_sizeW_/2); //中心点cell的位置 左下角的cell为位置为(0,0)
    gm_centerY_ = (int)(gm_sizeW_/2);
    gm_resolution_ = 0.05; //每个cell边长0.01m
    gm_locc_ =  0.9;
    gm_lfree_ = -0.85;
    gm_thickness_ = 1;
    //Motion
    m_a1_ = 0.05;
    m_a2_ = 5;
    m_a3_ = 0.05;
    m_a4_ = 0.0001;
    m_sigmaX_ = 0.001;
    m_sigmaY_ = 0.001;
    m_sigmaTh_= 1;
    //Observation
    o_sigma_ = 0.05;
    o_missLikelihood_ = -5.0;
    o_n_ = 17;
    o_kernel_size_ = 1;
    o_likelihood_threshold_ = 0.8;
    //ScanMatcher
    sm_translation_step_ = 0.2;//m
    sm_angle_step_ = 6.0; //角度
    sm_iteration_ = 5; //初始化迭代次数
    sm_delta_ = 10;
    sm_kernel_size_ = 1;
    sm_likelihood_threshold_ = 0.8;
    sm_debug_ = false;
    //ParticleFilter
    num_partices_ = 40;
    threshold_distance_ = 0.1;
    threshold_angle_ = 10*M_PI/180;
    threshold_hit_ = 40;

    GridMap map(gm_sizeW_, gm_sizeH_,gm_resolution_,gm_locc_,gm_lfree_,gm_thickness_);
    map.setCenter(gm_centerX_, gm_centerY_);
    Motion mo(m_a1_, m_a2_, m_a3_, m_a4_, m_sigmaX_, m_sigmaY_, m_sigmaTh_);
    Observation ob(o_sigma_, o_missLikelihood_, o_n_, o_kernel_size_, o_likelihood_threshold_);
    ScanMatcher sm(sm_translation_step_, sm_angle_step_, sm_iteration_, sm_delta_, sm_kernel_size_, sm_likelihood_threshold_,sm_debug_);

    pf_ = ParticleFilter(map, mo, ob, sm);

    //test initParticleFilter time consuming
    rclcpp::Time current_time, last_time;
    last_time = this->get_clock()->now();
    pf_.initParticleFilter(num_partices_, threshold_distance_, threshold_angle_, threshold_hit_);
    current_time = this->get_clock()->now();
    rclcpp::Duration dt = current_time - last_time;
    RCLCPP_INFO(this->get_logger(),"init ParticleFilter time consuming: %ld nanoseconds", dt.nanoseconds());

    //publish map and pose infomation
    occ_thresh_  = 0.01;
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    mapmd_publisher_ = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata",10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_pose",10);

    //tf
    odom_frame_ = "odom";
    map_frame_ = "map";
    base_link_frame_ = "base_link";
    transform_publish_period_ = 0.05;
    tf2_delay_ = transform_publish_period_;
}

void GridMappingRos::liveGmapping(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) 
{
    // RCLCPP_INFO(this->get_logger(),
    //         "I heard and synchronized the following timestamps: %u, %u",
    //         odom->header.stamp.sec, scan->header.stamp.sec);

    laser_frame_ =  scan->header.frame_id;
    double angle_center = 0; //0: laser坐标系x轴正方向与激光安装方向正前方向相重合。 (scan->angle_min + scan->angle_max)/2
    
    // centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
    //                                                           tf::Vector3(0,0,0)), scan->header.stamp, laser_frame_);
    centered_laser_pose_.header.stamp = scan->header.stamp;
    centered_laser_pose_.header.frame_id = laser_frame_;
    tf2::Vector3 vec3(0,0,0);
    centered_laser_pose_.pose.position.x = vec3.getX();
    centered_laser_pose_.pose.position.y = vec3.getY();
    centered_laser_pose_.pose.position.z = vec3.getZ();
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    centered_laser_pose_.pose.orientation.x = quat.x();
    centered_laser_pose_.pose.orientation.y = quat.y();
    centered_laser_pose_.pose.orientation.z = quat.z();
    centered_laser_pose_.pose.orientation.w = quat.w();


    if(getOdomPose(odom,scan->header.stamp)){
        //initPose
        //gmapping initpose 设置为最开始接收到的odom值
        if(count == 0){
            gmap_initPose = gmap_odomPose;
            count++;
        }

        getRangeReading(scan);
        pf_.doParticleFilter(gmap_initPose, rangeReading, gmap_odomPose);
        pf_.getBestParticle(targetParticle);
        publishMapInfo();

        //get the tf2 map to odom
        grid_mapping::Pose mPose = targetParticle.getTraceLast();
        // tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mPose.th), tf::Vector3(mPose.x, mPose.y, 0.0)).inverse();
        // tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, gmap_odomPose.th), tf::Vector3(gmap_odomPose.x, gmap_odomPose.y, 0.0));
        tf2::Quaternion q;
        q.setRPY(0, 0, mPose.th);
        tf2::Transform laser_to_map = tf2::Transform(q, tf2::Vector3(mPose.x, mPose.y, 0.0)).inverse();
        q.setRPY(0, 0, gmap_odomPose.th);
        tf2::Transform odom_to_laser = tf2::Transform(q, tf2::Vector3(gmap_odomPose.x, gmap_odomPose.y, 0.0));
        map_to_odom_mutex_.lock();
        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        map_to_odom_mutex_.unlock();
    }
}

bool GridMappingRos::getOdomPose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, const rclcpp::Time& t){
    centered_laser_pose_.header.stamp = t;
    geometry_msgs::msg::PoseStamped base_pose;
    try
    {
        //transformPose(base_link_frame_, centered_laser_pose_, base_pose);
        tf2_buffer_->transform(centered_laser_pose_, base_pose, base_link_frame_, tf2::durationFromSec(1.0));
    }
    catch(...)
    {
        // just continue on
    }
    /**
    * tf
     * 传递给pf的是laser在odom（map）坐标系下的坐标
    * **/
    geometry_msgs::msg::PoseStamped laserPose_inOdom;
    try{
        // transformPose(odom_frame_, base_pose, laserPose_inOdom);
        tf2_buffer_->transform(base_pose, laserPose_inOdom, odom_frame_, tf2::durationFromSec(1.0));
    }
    catch(...){
        return false;
    }

    gmap_odomPose.x = laserPose_inOdom.pose.position.x;
    gmap_odomPose.y = laserPose_inOdom.pose.position.y;
    /**
     * yaw的取值范围： 0~M_PI   0~-M_PI
     ***/
    // double yaw = laserPose_inOdom.pose.orientation.w;
    double yaw = tf2::getYaw(laserPose_inOdom.pose.orientation);
    //debug
    if(yaw < -M_PI || yaw > M_PI){
        RCLCPP_INFO(this->get_logger(),"debug test..................");
    }
    gmap_odomPose.th = yaw;

    return true;

}

void GridMappingRos::getRangeReading(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan){
    rangeReading.angle_min = scan->angle_min;
    rangeReading.angle_max = scan->angle_max;
    rangeReading.angle_increment = scan->angle_increment;
    rangeReading.range_min = scan->range_min;
    rangeReading.range_max = scan->range_max;
    rangeReading.range_count = scan->ranges.size();

//    double* ranges_double = new double[scan->ranges.size()];
    //得到scan数据
    rangeReading.ranges.resize(rangeReading.range_count);
    for(int i=0; i < rangeReading.range_count; i++)
    {
        rangeReading.ranges[i] = (double)scan->ranges[i];
    }
//    delete[] ranges_double;
}

void GridMappingRos::publishMapInfo(){

    bestCells = targetParticle.getMap().getCells();
    //publish map
    map_.map.info.resolution = gm_resolution_;
    int gm_sizew = targetParticle.getMap().getSizeW();
    int gm_sizeh = targetParticle.getMap().getSizeH();
    map_.map.info.width = gm_sizew;
    map_.map.info.height = gm_sizeh;
    map_.map.info.origin.position.x = -targetParticle.getMap().getCenter().x*gm_resolution_;
    map_.map.info.origin.position.y = -targetParticle.getMap().getCenter().y*gm_resolution_;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;

    map_.map.info.map_load_time = this->get_clock()->now();

    map_.map.header.stamp = this->get_clock()->now();
    map_.map.header.frame_id = map_frame_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    for(int i=0; i < map_.map.info.height; i++)
    {
        for(int j=0; j < map_.map.info.width; j++)
        {
            double occ = bestCells(i,j);
            /**
             * map_.map.data的数据格式是以左下角为[0]点，向右向上按行读取
             * ***/
//            assert(occ <= 1.0);
            if(occ < 0)
                map_.map.data[MAP_IDX(map_.map.info.width,map_.map.info.height, i, j)] = 0;
            else if(occ > occ_thresh_)
            {
                map_.map.data[MAP_IDX(map_.map.info.width,map_.map.info.height, i, j)] = 100;
            }
            else
                map_.map.data[MAP_IDX(map_.map.info.width,map_.map.info.height, i, j)] = -1;
        }
    }

    map_publisher_->publish(map_.map);
    mapmd_publisher_->publish(map_.map.info);
    RCLCPP_INFO(this->get_logger(),"Already publish map ...");
}

//publish tf: odom -> map
void GridMappingRos::publishLoop(double transform_publish_period){
    if(transform_publish_period == 0)
        return;

    rclcpp::Rate r(1.0 / transform_publish_period);
    while(rclcpp::ok()){
        publishTransform();
        r.sleep();
    }
}
void GridMappingRos::publishTransform()
{
    map_to_odom_mutex_.lock();
    rclcpp::Time tf_expiration = this->get_clock()->now()+ rclcpp::Duration(static_cast<int32_t>(static_cast<rcl_duration_value_t>(tf2_delay_)), 0);
    //tfB_.sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = map_frame_;
    transform.header.stamp = tf_expiration;
    transform.child_frame_id = odom_frame_;
    try {
        transform.transform = tf2::toMsg(map_to_odom_);
        tfB_->sendTransform(transform);
    }
    catch (tf2::LookupException& te){
        RCLCPP_ERROR(this->get_logger(), te.what());
    }
    map_to_odom_mutex_.unlock();
}

bool GridMappingRos::transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose, 
    geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf2_buffer_->transform(
      in_pose, out_pose, frame,
      tf2::durationFromSec(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Exception in transformPose: %s", ex.what());
  }
  return false;
}