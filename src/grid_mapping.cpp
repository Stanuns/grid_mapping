#include "rclcpp/rclcpp.hpp"
#include "grid_mapping/grid_mapping_ros.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMappingRos>());
  rclcpp::shutdown();
  return 0;
}