#pragma once

#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <vrobot_mission/constance.hpp>
#include <vrobot_mission/msg/action_state.hpp>
#include <vrobot_mission/srv/run_mission.hpp>
#include <vrobot_mission/utils/node_thread.hpp>
#include <vrobot_mission/utils/service_client.hpp>

#include <std_srvs/srv/empty.hpp>

#include <boost/process.hpp>

using std_srvs::srv::Empty;
using std_srvs::srv::Trigger;
using vrobot_mission::msg::ActionState;
using vrobot_mission::srv::RunMission;

using vrobot_mission::ServiceClient;

namespace vrobot_mission {
class VrobotMission : public rclcpp::Node {
public:
  VrobotMission(const rclcpp::NodeOptions &options);
  ~VrobotMission();

private:
  inline std::string indent_lines(const std::string &pycode,
                                  const std::string &prefix) {
    std::stringstream input(pycode);
    std::stringstream output;
    std::string       line;

    // Read line-per-line and add indent prefix
    while (std::getline(input, line)) {
      output << prefix << line << '\n';
    }

    return output.str();
  }

protected:
  bool init_services();
  bool init_subscribers();
  bool init_publishers();
  bool init_timers();

private:
  bool stop_mission();
  bool cancel_move_to_pose();

private:
  rclcpp::Node::SharedPtr      mission_node_;
  rclcpp::Node::SharedPtr      cancel_mission_node_;
  std::shared_ptr<std::thread> mission_thread_;
  std::mutex                   mission_thread_mutex_;
  boost::process::child        process_child_;
  std::atomic_bool             is_processing_{false};
  ActionState                  action_state_;
  void                         thread_mission_child(const std::string &pycode);
  void                         update_action_state(std::string state);

  rclcpp::Service<RunMission>::SharedPtr run_mission_service_;
  NodeThread::Ptr                        run_mission_thread_;
  rclcpp::Service<Trigger>::SharedPtr    stop_mission_service_;
  NodeThread::Ptr                        stop_mission_thread_;

  // Cancel mission
  std::shared_ptr<ServiceClient<Empty>> cancel_move_to_pose_client_{nullptr};

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