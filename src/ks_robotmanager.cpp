#include "ks_robotmanager.h"

#include <optional>

#include "interface/ks_api.h"
#include "utilities.h"

namespace ks {

using namespace std;

void KsRobotManager::Init() {
  const vector<Location> &shelf_storage_points = ks_map_.GetShelfStoragePoints();
  for (int i = 0; i < ks_map_.robot_count_; i++) {
    robot_info_.emplace_back(i, shelf_storage_points[i]);
  }
}

vector<RobotInfo *> KsRobotManager::GetIdleRobots() {
  vector<RobotInfo *> rtn;
  for (RobotInfo &r : robot_info_) {
    if (r.IsIdle()) {
      rtn.push_back(&r);
    }
  }
  return rtn;
}

// Action @a is done for robot @robot_id, update robot status.
std::optional<MissionReport> KsRobotManager::UpdateRobotStatus(int robot_id, Action a) {
  RobotInfo &robot = robot_info_[robot_id];

  ApplyActionOnRobot(a, &robot);
  if (a == Action::ATTACH) {
    return MissionReport(robot.mission.wms_mission, MissionReportType::PICKUP_DONE);
  } else if (a == Action::DETACH) {
    return MissionReport(robot.mission.wms_mission, MissionReportType::MISSION_DONE);
  } else {
    return nullopt;
  }
}

void KsRobotManager::AssignMissions(std::set<WmsMission> &missions) {
  auto mission_it = missions.begin();
  while (mission_it != missions.end()) {
    WmsMission to_assign = *mission_it;
    if (!IsMissionValid(to_assign)) {
      mission_it++;
      continue;
    }
    if (HasIdleRobot()) {
      RobotInfo* picked_robot = GetIdleRobotAtLocation(to_assign.pick_from.loc);
      if (picked_robot == nullptr) {
        picked_robot = GetClosestIdleRobot(to_assign.pick_from.loc);
      }

      picked_robot->has_mission = true;
      picked_robot->mission.is_internal = false;
      picked_robot->mission.wms_mission = to_assign;

      mission_it = missions.erase(mission_it);
    } else {
      break;
    }
  }

  vector<RobotInfo *> idle_robots = GetIdleRobots();
  set<Location> free_locations = GetFreeLocations(missions);
  for (RobotInfo* r : idle_robots) {
    // If r is at the destination of any unassigned mission m, then move r to a random storage location
    // that is not 1. occupied, 2. is the source or dest of any mission.
    for (const WmsMission& mission : missions) {
      if (r->pos.loc == mission.drop_to.loc) {
        r->has_mission = true;
        r->mission.is_internal = true;
        r->mission.internal_mission.to = *free_locations.begin();
        free_locations.erase(free_locations.begin());
        cout << "Assigned internal mission to robot: " << r->id
            << " move to: " << r->mission.internal_mission.to.to_string() << endl;
      }
    }
  }
}

RobotInfo *KsRobotManager::GetIdleRobotAtLocation(Location loc) {
  vector<RobotInfo *> idle_robots = GetIdleRobots();
  for (RobotInfo *p : idle_robots) {
    if (p->pos.loc == loc) {
      return p;
    }
  }
  return nullptr;
}

RobotInfo *KsRobotManager::GetClosestIdleRobot(Location loc) {
  vector<RobotInfo *> idle_robots = GetIdleRobots();
  if (idle_robots.empty()) {
    return nullptr;
  }

  RobotInfo *picked_robot = idle_robots[0];
  int dist = GetManhattanDist(picked_robot->pos.loc, loc);
  for (int i = 1; i < (int)idle_robots.size(); i++) {
    int new_dist = GetManhattanDist(idle_robots[i]->pos.loc, loc);
    if (dist > new_dist) {
      dist = new_dist;
      picked_robot = idle_robots[i];
    }
  }
  return picked_robot;
}

bool KsRobotManager::IsMissionValid(const WmsMission &mission) {
  const Location& pickup_loc = mission.pick_from.loc;
  const Location& dropdown_loc = mission.drop_to.loc;
  for (const RobotInfo& r : robot_info_) {
    if (!r.has_mission) {
      continue;
    }
    const Mission& assigned = r.mission;
    if (assigned.is_internal) {
      if (assigned.internal_mission.to == pickup_loc || assigned.internal_mission.to == dropdown_loc) {
        return false;
      }
    } else {
      // Does not need to check the wms mission case, since WMS guarantees the pickup and drop down location
      // won't overlap.
    }
  }

  // If there is a free robot at the mission's pickup location, then this mission is valid.
  for (const RobotInfo& r : robot_info_) {
    if (r.IsIdle() && (r.pos.loc == pickup_loc)) {
      return true;
    }
  }

  // Except for the previous case, there cannot be robot at pickup/dropdown location of a mission.
  for (const RobotInfo& r : robot_info_) {
    if (r.pos.loc == pickup_loc || r.pos.loc == dropdown_loc) {
      return false;
    }
  }

  return true;
}

// A free location is defined by:
// 1. Not the current position of any robot.
// 2. Not the end points of any assigned mission.
// 3. Not the end points of any to be assigned mission.
std::set<Location> KsRobotManager::GetFreeLocations(const std::set<WmsMission> &missions) {
  std::set<Location> rtn(ks_map_.GetShelfStoragePoints().begin(),
      ks_map_.GetShelfStoragePoints().end());

  for (const RobotInfo& r : robot_info_) {
    rtn.erase(r.pos.loc);
    if (r.has_mission) {
      if (r.mission.is_internal) {
        rtn.erase(r.mission.internal_mission.to);
      } else {
        rtn.erase(r.mission.wms_mission.pick_from.loc);
        rtn.erase(r.mission.wms_mission.drop_to.loc);
      }
    }
  }

  for (const WmsMission& m : missions) {
    rtn.erase(m.pick_from.loc);
    rtn.erase(m.drop_to.loc);
  }

  assert(rtn.size() > 0);
  return rtn;
}

}