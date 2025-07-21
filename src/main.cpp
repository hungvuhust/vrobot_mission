#include <rclcpp/rclcpp.hpp>
#include <vrobot_mission/vrobot_mission.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<vrobot_mission::VrobotMission>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
