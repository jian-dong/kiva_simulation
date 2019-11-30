#ifndef KIVA_SIMULATION_SRC_KS_SCHEDULER_COMMON_H_
#define KIVA_SIMULATION_SRC_KS_SCHEDULER_COMMON_H_

#include <iostream>

#include "constants.h"
#include "utilities.h"
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
  explicit Mission(WmsMission wms_mission) : wms_mission(wms_mission), is_internal(false) {};
  explicit Mission(InternalMission internal_mission) : internal_mission(internal_mission), is_internal(true) {};
  Mission &operator=(const Mission &o) = default;

  bool operator==(const Mission &o) const {
    if (is_internal != o.is_internal) {
      return false;
    }
    if (is_internal) {
      return internal_mission.to == o.internal_mission.to;
    } else {
      return wms_mission.id == o.wms_mission.id;
    }
  }
};

struct Position {
  Location loc;
  Direction dir;

  Position() = default;
  Position(int x, int y, Direction dir) : loc(x, y), dir(dir) {};
  Position(Location loc, Direction dir) : loc(loc), dir(dir) {};

  bool operator<(const Position &o) const {
    if (loc != o.loc) {
      return loc < o.loc;
    }
    return dir < o.dir;
  }

  bool operator==(const Position &o) const {
    return loc == o.loc && dir == o.dir;
  }

  bool operator!=(const Position &o) const {
    return !operator==(o);
  }

  [[nodiscard]] std::string to_string() const {
    return loc.to_string() + " " + kDirectionToString.at(dir);
  }
};

struct RobotInfo {
  int id;
  Position pos;
  bool shelf_attached;
  bool has_mission;
  Mission mission;

  RobotInfo() = default;
  RobotInfo(int id, Location loc) : id(id) {
    pos = {loc, Direction::NORTH};
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

  std::string to_string() const {
    return "id: " + std::to_string(id)
    + " pos: " + pos.to_string()
    + " has mission: " + std::to_string(has_mission);
  }

  bool operator==(const RobotInfo& o) const {
    return id == o.id
    && pos == o.pos
    && shelf_attached == o.shelf_attached
    && has_mission == o.has_mission
    && mission == o.mission;
  }
};

struct ActionWithTime {
  Action action;
  int start_time_ms;
  int end_time_ms;
  Position start_pos;
  Position end_pos;

  ActionWithTime() = default;
  ActionWithTime(Action action, int start_time_ms, int end_time_ms, Position start_pos, Position end_pos)
      : action(action), start_time_ms(start_time_ms), end_time_ms(end_time_ms),
        start_pos(start_pos), end_pos(end_pos) {};

  [[nodiscard]] std::string to_string() const {
    return kActionToString.at(action)
        + " start: " + std::to_string(start_time_ms)
        + " end: " + std::to_string(end_time_ms);
  }
};

using ActionWithTimeSeq = std::vector<ActionWithTime>;
using ActionPlan = std::vector<ActionWithTimeSeq>;

class ShelfManager {
 public:
  ShelfManager() = default;
  ShelfManager(const ShelfManager &o) {
    loc_to_id_ = o.loc_to_id_;
  }
  ShelfManager &operator=(const ShelfManager &o) {
    loc_to_id_ = o.loc_to_id_;
  }

  void AddMapping(int shelf_id, Location loc) {
    assert(loc_to_id_.find(loc) == loc_to_id_.end());
    loc_to_id_[loc] = shelf_id;
  }

  void RemoveMapping(int shelf_id, Location loc) {
    assert(loc_to_id_.find(loc) != loc_to_id_.end());
    assert(loc_to_id_[loc] == shelf_id);
    loc_to_id_.erase(loc);
  }

  [[nodiscard]] bool HasShelf(Location loc) const {
    return loc_to_id_.find(loc) != loc_to_id_.end();
  }

  std::map<Location, int> loc_to_id_;
};

// Helper functions.
inline void ApplyActionOnRobot(Action a, RobotInfo *r, ShelfManager* sm) {
  switch (a) {
    case Action::ATTACH:assert(!r->shelf_attached);
      assert(r->has_mission);
      assert(!r->mission.is_internal);
      r->shelf_attached = true;
      if (sm != nullptr) {
        sm->RemoveMapping(r->mission.wms_mission.shelf_id,
                          r->mission.wms_mission.pick_from.loc);
      }
      break;
    case Action::DETACH:assert(r->shelf_attached);
      assert(r->has_mission);
      assert(!r->mission.is_internal);
      r->shelf_attached = false;
      r->has_mission = false;
      if (sm != nullptr) {
        sm->AddMapping(r->mission.wms_mission.shelf_id,
                          r->mission.wms_mission.drop_to.loc);
      }
      break;
    case Action::YIELD:
      // Yield is effectively a WAIT operation in the MAPF setting. Do nothing.
      break;
    case Action::MOVE:r->pos.loc = r->pos.loc + kDirectionToDelta.at(r->pos.dir);
      break;
    case Action::CTURN:r->pos.dir = ClockwiseTurn(r->pos.dir);
      break;
    case Action::CCTURN:r->pos.dir = CounterClockwiseTurn(r->pos.dir);
      break;
    default:exit(0);
  }
  if (r->has_mission && r->mission.is_internal && r->pos.loc == r->mission.internal_mission.to) {
    r->has_mission = false;
  }
}

inline void ApplyActionOnPosition(Action a, Position* p) {
  switch (a) {
    case Action::ATTACH:break;
    case Action::DETACH:break;
    case Action::YIELD:break;
    case Action::MOVE:p->loc = p->loc + kDirectionToDelta.at(p->dir);
      break;
    case Action::CTURN:p->dir = ClockwiseTurn(p->dir);
      break;
    case Action::CCTURN:p->dir = CounterClockwiseTurn(p->dir);
      break;
    default:exit(0);
  }
}

// Debug helpers.
inline void PrintRobotInfo(const std::vector<RobotInfo> &robot_info) {
  int robot_count = robot_info.size();
  for (int i = 0; i < robot_count; i++) {
    std::cout << robot_info[i].to_string() << std::endl;
  }
}
}
#endif