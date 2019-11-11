#include "sipp_solver.h"

namespace ks {
using std::vector;

namespace {
double GetActionCostInTime(Action a) {
  // TODO: implement this.
  return 0;
}

void AppendToVector() {

}
}

PfResponse SippSolver::FindPath(const PfRequest &req) {
  // 1. Initialize all safe intervals, for robots with no mission, the location they stay is always non-safe.
  safe_intervals_.clear();
  const vector<Location>& passable = map_.GetPassableLocations();
  for (const Location& l : passable) {
    safe_intervals_[l].emplace_back(0, kDoubleInf);
  }
  for (int i = 0; i < kRobotCount; i++) {
    if (!req.robots[i].has_mission) {
      safe_intervals_[req.robots[i].loc].clear();
    }
  }

  // 2. For each robot, plan a route.
  PfResponse rtn;
  for (int robot_id = 0; robot_id < kRobotCount; robot_id++) {
    const RobotInfo& robot = req.robots[robot_id];
    if (!robot.has_mission) {
      continue;
    }

    if (robot.mission.is_internal) {
      PlanInternalMission(robot);
    } else {
      PlanWmsMission(robot);
    }
  }

  return {};
}
ActionSeqWithTime SippSolver::GetActions(double start_time, bool has_shelf, Location src, Location target) {
  return ActionSeqWithTime();
}

void SippSolver::PlanInternalMission(const RobotInfo& robot) {

}

void SippSolver::PlanWmsMission(const RobotInfo& robot) {
  // Plan for the first half.
//  float second_half_start_time = 0;
//  if (!robot.shelf_attached) {
//    const ActionSeqWithTime& actions = GetActions(0, false, robot.loc, robot.mission.from.loc);
//    rtn.actions[robot_id].actions.insert(rtn.actions[robot_id].actions.begin(),
//                                         a.actions.begin(), a.actions.end());
//    if (!rtn.actions[robot_id].actions.empty()) {
//      second_half_start_time = rtn.actions[robot_id].actions.back().first
//          + GetActionCostInTime(a.actions.back().second)
//          + GetActionCostInTime(Action::ATTACH);
//    }
//  }
//
//  // Plan for the second half.
//  const auto& a = GetActions(second_half_start_time, true, robot.loc, robot.mission.from.loc);

}
}