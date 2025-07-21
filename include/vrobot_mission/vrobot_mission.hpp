#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vrobot_mission/constance.hpp>
#include <vrobot_mission/msg/action_state.hpp>
#include <vrobot_mission/srv/run_mission.hpp>

using std_srvs::srv::Trigger;
using vrobot_mission::msg::ActionState;
using vrobot_mission::srv::RunMission;

namespace vrobot_mission {
class VrobotMission : public rclcpp::Node {
public:
  VrobotMission(const rclcpp::NodeOptions &options);
  ~VrobotMission();

protected:
  bool init_services();
  bool init_subscribers();
  bool init_publishers();
  bool init_timers();

private:
  rclcpp::Service<RunMission>::SharedPtr run_mission_service_;
  rclcpp::Service<Trigger>::SharedPtr    stop_mission_service_;

  void run_mission_callback(const std::shared_ptr<RunMission::Request> request,
                            std::shared_ptr<RunMission::Response> response);

  void stop_mission_callback(const std::shared_ptr<Trigger::Request> request,
                             std::shared_ptr<Trigger::Response>      response);

private:
  rclcpp::Publisher<ActionState>::SharedPtr action_state_publisher_;
  rclcpp::TimerBase::SharedPtr              action_state_timer_;
  void                                      on_action_state_timer();
};
} // namespace vrobot_mission