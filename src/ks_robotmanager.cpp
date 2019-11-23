#include "ks_robotmanager.h"

#include <optional>

#include "interface/ks_api.h"
#include "logger.h"
#include "utilities.h"

namespace ks {

using namespace std;

void KsRobotManager::Init() {
  const vector<Location> &shelf_storage_points = map_.GetShelfStoragePoints();
  for (int i = 0; i < kRobotCount; i++) {
    robot_info_.emplace_back(i, shelf_storage_points[i]);
  }

  rest_area_assignment_.resize(rest_areas_.size());
  for (int i = 0; i < kRestAreaCount; i++) {
    rest_area_assignment_[i] = -1;
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
  while (!missions.empty()) {
    WmsMission m = *missions.begin();

    if (!HasIdleRobots()) {
      return;
    }
    // There is at least one free robot, the task will be assigned.
    missions.erase(missions.begin());

    // TODO: This program assumes WMS won't schedule other robots to move to
    // the "from" location or the "to" location. Maybe have a check to verify this?
    RobotInfo *picked_robot = GetIdleRobotAtLocation(m.pick_from.loc);
    if (picked_robot == nullptr) {
      picked_robot = GetClosestIdleRobot(m.pick_from.loc);
    }
    assert(picked_robot != nullptr);
    FreeFromRestArea(picked_robot->id);
    picked_robot->has_mission = true;
    picked_robot->mission.is_internal = false;
    picked_robot->mission.wms_mission = m;

    RobotInfo *blocking_robot = GetIdleRobotAtLocation(m.drop_to.loc);
    if (blocking_robot != nullptr) {
      blocking_robot->has_mission = true;
      blocking_robot->mission.is_internal = true;
      blocking_robot->mission.internal_mission.to = AssignToRestArea(blocking_robot->id);
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
  for (int i = 1; i < idle_robots.size(); i++) {
    int new_dist = GetManhattanDist(idle_robots[i]->pos.loc, loc);
    if (dist > new_dist) {
      dist = new_dist;
      picked_robot = idle_robots[i];
    }
  }
  return picked_robot;
}

Location KsRobotManager::AssignToRestArea(int robot_id) {
  for (int i = 0; i < kRestAreaCount; i++) {
    if (rest_area_assignment_[i] == -1) {
      rest_area_assignment_[i] = robot_id;
      return rest_areas_[i];
    }
  }

  LogFatal("No free rest area.");
}

void KsRobotManager::FreeFromRestArea(int robot_id) {
  for (int i = 0; i < kRestAreaCount; i++) {
    if (rest_area_assignment_[i] == robot_id) {
      rest_area_assignment_[i] = -1;
      return;
    }
  }
}

}