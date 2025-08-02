#pragma once

#include <string>

namespace vrobot_mission {

const std::string kServiceRunMission  = "/vrobot/mission/run";
const std::string kServiceStopMission = "/vrobot/mission/stop";
const std::string kTopicActionState   = "/vrobot/mission/state";

const std::string kServiceCancelMoveToPose = "/vrobot/move/cancel";

constexpr char kPythonScriptPath[] =
    "/home/rtc/ros2_ws/src/vrobot_mission/scripts/tmp.py";
// Python code
constexpr char kPythonCodeHeader[] =
    "from service_cli import *\n"
    "import pdb\n"
    "import sys\n"
    "import time\n"
    "rclpy.init()\n"
    "robot = ServiceCli()\n"
    "pdb.set_trace()\n"
    "try:\n";

constexpr char kPythonCodeFooter[] =
    "\n"
    "except Exception as e:\n"
    "    print('_ERROR_')\n"
    "pdb.set_trace = lambda: None\n"
    "rclpy.shutdown() ";

} // namespace vrobot_mission