#include "vrobot_mission/constance.hpp"
#include <chrono>
#include <string>
#include <vrobot_mission/vrobot_mission.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace vrobot_mission {

VrobotMission::VrobotMission(const rclcpp::NodeOptions &options)
    : Node("vrobot_mission", options) {
  RCLCPP_INFO(this->get_logger(), "VrobotMission node initialized");

  mission_node_ = std::make_shared<rclcpp::Node>("mission_spin_node", options);
  cancel_mission_node_ =
      std::make_shared<rclcpp::Node>("cancel_mission_node", options);

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
  if (mission_node_) {
    run_mission_service_ = mission_node_->create_service<RunMission>(
        kServiceRunMission,
        std::bind(&VrobotMission::run_mission_callback, this, _1, _2));
  } else {
    throw std::runtime_error("Mission node not initialized");
  }

  if (cancel_mission_node_) {
    stop_mission_service_ = cancel_mission_node_->create_service<Trigger>(
        kServiceStopMission,
        std::bind(&VrobotMission::stop_mission_callback, this, _1, _2));

    cancel_move_to_pose_client_ = std::make_shared<ServiceClient<Empty>>(
        kServiceCancelMoveToPose, cancel_mission_node_);

  } else {
    throw std::runtime_error("Cancel mission node not initialized");
  }

  RCLCPP_INFO(this->get_logger(), "Services initialized");
  run_mission_thread_  = std::make_shared<NodeThread>(mission_node_);
  stop_mission_thread_ = std::make_shared<NodeThread>(cancel_mission_node_);
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
      500ms, std::bind(&VrobotMission::on_action_state_timer, this));
  return true;
}

void VrobotMission::run_mission_callback(
    const std::shared_ptr<RunMission::Request> request,
    std::shared_ptr<RunMission::Response>      response) {
  (void)request;
  {
    std::lock_guard<std::mutex> lock(mission_thread_mutex_);
    action_state_.action_id = request->mission_name;
    action_state_.state     = ActionState::WAITING;
  }
  try {
    RCLCPP_INFO(this->get_logger(), "Running mission");
    bool is_accept = !(is_processing_.load()) and !(process_child_.running());
    if (is_accept) {

      is_processing_.store(true);

      if (mission_thread_) {
        if (mission_thread_->joinable()) {
          mission_thread_->join();
          RCLCPP_INFO(this->get_logger(), "Mission thread joined");
        }
      }

      mission_thread_ = std::make_shared<std::thread>(std::bind(
          &VrobotMission::thread_mission_child, this, request->pycode));

      response->success = true;
    } else {
      response->success = false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error running mission: %s", e.what());
    stop_mission();
    response->success = false;
  }
}

void VrobotMission::stop_mission_callback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response>      response) {
  (void)request;
  try {
    RCLCPP_INFO(this->get_logger(), "Requesting to stop mission");
    if (is_processing_.load()) {
      stop_mission();
    }
    response->success = true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error stopping mission: %s", e.what());
    response->success = false;
  }
}

void VrobotMission::on_action_state_timer() {
  RCLCPP_DEBUG(this->get_logger(), "Publishing action state");
  std::lock_guard<std::mutex> lock(mission_thread_mutex_);
  action_state_publisher_->publish(action_state_);
}

bool VrobotMission::stop_mission() {
  RCLCPP_INFO(this->get_logger(), "Stopping mission");

  if (process_child_.running()) {
    // Đặt flag để báo hiệu dừng process
    is_processing_.store(false);

    // Cancel move to pose trước khi terminate process
    cancel_move_to_pose();

    // Join mission thread TRƯỚC khi terminate process để tránh broken pipe
    if (mission_thread_) {
      if (mission_thread_->joinable()) {
        // Đợi thread kết thúc với timeout
        auto start           = std::chrono::steady_clock::now();
        bool thread_finished = false;

        while (!thread_finished && std::chrono::steady_clock::now() - start <
                                       std::chrono::seconds(3)) {
          if (mission_thread_->joinable()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          } else {
            thread_finished = true;
          }
        }

        if (mission_thread_->joinable()) {
          try {
            mission_thread_->join();
            RCLCPP_INFO(this->get_logger(), "Mission thread joined");
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(),
                        "Exception while joining mission thread: %s", e.what());
          }
        }
      }
    }

    // Bây giờ mới terminate process sau khi thread đã dừng
    if (process_child_.running()) {
      try {
        // Đợi một chút để process tự dừng
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (process_child_.running()) {
          // Thử gửi SIGTERM trước (graceful shutdown)
          try {
            process_child_.terminate();
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(),
                        "Exception while sending SIGTERM: %s", e.what());
          }

          // Đợi process terminate với timeout
          auto start = std::chrono::steady_clock::now();
          while (process_child_.running() &&
                 std::chrono::steady_clock::now() - start <
                     std::chrono::seconds(2)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }

          // Xử lý kết quả
          if (process_child_.running()) {
            process_child_.detach();
            RCLCPP_WARN(this->get_logger(), "Process forcefully detached");
          } else {
            try {
              process_child_.wait();
              RCLCPP_INFO(this->get_logger(), "Process child terminated");
            } catch (const std::exception &e) {
              RCLCPP_WARN(this->get_logger(),
                          "Exception while waiting for process: %s", e.what());
            }
          }
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "Exception during process termination: %s", e.what());
      }
    }
  }

  return true;
}

void VrobotMission::thread_mission_child(const std::string &pycode) {
  RCLCPP_INFO(this->get_logger(), "Thread mission child");
  update_action_state(ActionState::INITIALIZING);
  std::string python_code = kPythonCodeHeader;
  std::string code_indent = indent_lines(pycode.c_str(), "    ");
  python_code += code_indent;
  python_code += kPythonCodeFooter;

  std::string   dir = std::string(kPythonScriptPath);
  std::ofstream temp_file(dir);
  temp_file << python_code;
  temp_file.close();
  boost::process::opstream pipe_in;
  boost::process::ipstream pipe_out;

  process_child_ = boost::process::child(
      "/bin/python3 " + dir,
      boost::process::std_in<pipe_in, boost::process::std_out> pipe_out,
      boost::process::std_err > stderr);

  std::string line;
  bool        is_exception = false;
  bool        is_cancelled = false;

  while (std::getline(pipe_out, line)) {
    if (is_processing_.load() == false) {
      try {
        if (pipe_in.good()) {
          pipe_in << "q\n" << std::flush;
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Exception while sending quit command: %s",
                    e.what());
      }
      is_cancelled = true;
      break;
    }
    RCLCPP_INFO(get_logger(), "Received from pdb: %s", line.c_str());
    try {
      if (line.find("-> pdb.set_trace = lambda: None") != std::string::npos) {
        if (pipe_in.good()) {
          pipe_in << "c\n" << std::flush;
        }
      } else if (line.find("->") != std::string::npos) {
        if (pipe_in.good()) {
          pipe_in << "n\n" << std::flush;
        }
        update_action_state(ActionState::RUNNING);
      } else if (line.find("_ERROR_") != std::string::npos) {
        is_exception = true;
        if (pipe_in.good()) {
          pipe_in << "n\n" << std::flush;
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(),
                  "Exception while communicating with process: %s", e.what());
      break;
    }
  }

  // Đóng pipe một cách rõ ràng để tránh broken pipe
  try {
    // Flush và đóng input pipe trước
    if (pipe_in.good()) {
      pipe_in.flush();
      pipe_in.close();
    }
    // Đóng output pipe
    if (pipe_out.good()) {
      pipe_out.close();
    }
    RCLCPP_DEBUG(get_logger(), "Pipes closed successfully");
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Exception while closing pipes: %s", e.what());
  }

  // Đợi process kết thúc một cách an toàn
  try {
    if (process_child_.running()) {
      process_child_.wait();
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Exception while waiting for process: %s",
                e.what());
  }

  if (is_cancelled) {
    update_action_state(ActionState::CANCELLED);
  } else if (is_exception) {
    update_action_state(ActionState::FAILED);
  } else {
    update_action_state(ActionState::FINISHED);
  }

  std::this_thread::sleep_for(1s);
  is_processing_.store(false);

  RCLCPP_INFO(this->get_logger(), "Finished thread mission child");
}

void VrobotMission::update_action_state(std::string state) {
  std::lock_guard<std::mutex> lock(mission_thread_mutex_);
  action_state_.state = state;
}

bool VrobotMission::cancel_move_to_pose() {
  RCLCPP_INFO(this->get_logger(), "Cancelling move to pose");

  if (cancel_move_to_pose_client_) {
    auto request  = std::make_shared<Empty::Request>();
    auto response = cancel_move_to_pose_client_->invoke(request, 5000ms);
    if (response) {
      RCLCPP_INFO(this->get_logger(), "Move to pose cancelled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to cancel move to pose");
      return false;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Cancel move to pose client not initialized");
    return false;
  }

  return true;
}

} // namespace vrobot_mission