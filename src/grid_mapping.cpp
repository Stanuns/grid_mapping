/*
 * @Author: Wei Sun 
 * @Date: 2024-07-09 09:15:51 
 * @Last Modified by: Wei Sun
 * @Last Modified time: 2024-07-11 09:55:21
 */
#include "rclcpp/rclcpp.hpp"
#include "grid_mapping/grid_mapping_ros.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMappingRos>());
  rclcpp::shutdown();
  return 0;
}