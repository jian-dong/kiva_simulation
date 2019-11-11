#ifndef KIVA_SIMULATION_SRC_KS_SCHEDULER_COMMON_H_
#define KIVA_SIMULATION_SRC_KS_SCHEDULER_COMMON_H_

#include "constants.h"
#include "interface/wms_scheduler_types.h"

namespace ks {
struct InternalMission {
  // Internal mission, for a robot without shelf moving around.
  Location to;
};

struct Mission {
  bool is_internal;
  WmsMission wms_mission;
  InternalMission internal_mission;
  Mission() = default;
};

struct RobotInfo {
  int id;
  Direction dir;
  Location loc;
  bool shelf_attached;

  // When has_mission is false, and report_mission_done is true, the robot has
  // finished the mission, and the scheduler needs to update this to wms.
  bool has_mission;
  Mission mission;

  RobotInfo(int id, Location loc) : id(id), loc(loc){
    dir = Direction::NORTH;
    shelf_attached = false;
    has_mission = false;
  }

  RobotInfo(const RobotInfo &o) = default;
  RobotInfo &operator=(const RobotInfo &o) = default;

  bool IsIdle() const {
    bool rtn = !has_mission;
    if (rtn) {
      assert(!shelf_attached);
    }
    return rtn;
  }
};

inline void ApplyActionOnRobot(Action a, RobotInfo* r) {
  switch (a) {
    case Action::ATTACH:assert(r->shelf_attached == false);
      r->shelf_attached = true;
      break;
    case Action::DETACH:assert(r->shelf_attached == true);
      r->shelf_attached = false;
      r->has_mission = false;
      break;
    case Action::YIELD:
      // Yield is effectively a WAIT operation in the MAPF setting. Do nothing.
      break;
    case Action::MOVE:r->loc = r->loc + kDirectionToDelta.at(r->dir);
      break;
    case Action::CTURN:r->dir = ClockwiseTurn(r->dir);
      break;
    case Action::CCTURN:r->dir = CounterClockwiseTurn(r->dir);
      break;
    default:exit(0);
  }
}

struct ActionSeqWithTime {
  // A sequence of actions, along with the start time of each action.
  std::vector<std::pair<double, Action>> actions;
};

}
#endif