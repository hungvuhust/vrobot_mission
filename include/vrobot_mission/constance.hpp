#pragma once

#include <string>

namespace vrobot_mission {

const std::string kServiceRunMission  = "/vrobot_mission/run";
const std::string kServiceStopMission = "/vrobot_mission/stop";
const std::string kTopicActionState   = "/vrobot_mission/state";

} // namespace vrobot_mission