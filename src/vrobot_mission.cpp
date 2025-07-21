#include <vrobot_mission/vrobot_mission.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace vrobot_mission {

VrobotMission::VrobotMission(const rclcpp::NodeOptions &options)
    : Node("vrobot_mission", options) {
  RCLCPP_INFO(this->get_logger(), "VrobotMission node initialized");
  init_services();
  init_subscribers();
  init_publishers();
  init_timers();
}

VrobotMission::~VrobotMission() {
  RCLCPP_INFO(this->get_logger(), "VrobotMission node destroyed");
}

bool VrobotMission::init_services() {
  RCLCPP_INFO(this->get_logger(), "Initializing services");
  run_mission_service_ = this->create_service<RunMission>(
      kServiceRunMission,
      std::bind(&VrobotMission::run_mission_callback, this, _1, _2));
  stop_mission_service_ = this->create_service<Trigger>(
      kServiceStopMission,
      std::bind(&VrobotMission::stop_mission_callback, this, _1, _2));
  RCLCPP_INFO(this->get_logger(), "Services initialized");
  return true;
}

bool VrobotMission::init_subscribers() {
  RCLCPP_INFO(this->get_logger(), "Initializing subscribers");
  return true;
}

bool VrobotMission::init_publishers() {
  RCLCPP_INFO(this->get_logger(), "Initializing publishers");
  action_state_publisher_ =
      this->create_publisher<ActionState>(kTopicActionState, 10);
  RCLCPP_INFO(this->get_logger(), "Publishers initialized");
  return true;
}

bool VrobotMission::init_timers() {
  RCLCPP_INFO(this->get_logger(), "Initializing timers");
  action_state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&VrobotMission::on_action_state_timer, this));
  return true;
}

void VrobotMission::run_mission_callback(
    const std::shared_ptr<RunMission::Request> request,
    std::shared_ptr<RunMission::Response>      response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Running mission");
  response->success = true;
}

void VrobotMission::stop_mission_callback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response>      response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Stopping mission");
  response->success = true;
}

void VrobotMission::on_action_state_timer() {
  RCLCPP_DEBUG(this->get_logger(), "Publishing action state");
  action_state_publisher_->publish(ActionState());
}

} // namespace vrobot_mission