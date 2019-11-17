#include "sipp_solver.h"

#include "logger.h"
#include "sipp_astar.h"

namespace ks {

using namespace std;
using sipp_astar::SippAstar;

namespace {
template<class T>
void AppendToVector(const vector<T> &from, vector<T> *to) {
  to->insert(to->end(), from.begin(), from.end());
}
}

PfResponse SippSolver::FindPath(const PfRequest &req) {
  // 1. Initialize all safe intervals, for robots with no mission, the location they stay is always non-safe.
  robot_count_ = req.robots.size();
  safe_intervals_.clear();
  const vector<Location> &passable = map_.GetPassableLocations();
  for (const Location &l : passable) {
    safe_intervals_[l] = IntervalSeq();
  }
  for (int i = 0; i < robot_count_; i++) {
    if (!req.robots[i].has_mission) {
      safe_intervals_[req.robots[i].pos.loc].Clear();
    }
  }

  // 2. For each robot, plan a route.
  PfResponse rtn;
  rtn.plan.resize(robot_count_);
  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    const RobotInfo &robot = req.robots[robot_id];
    if (!robot.has_mission) {
      continue;
    }

    if (robot.mission.is_internal) {
      PlanInternalMission(robot, &rtn.plan[robot_id]);
    } else {
      PlanWmsMission(robot, &rtn.plan[robot_id]);
    }
  }

  return rtn;
}

void SippSolver::PlanInternalMission(const RobotInfo &robot, ActionWithTimeSeq *rtn) {
  SippAstar astar(map_, safe_intervals_, shelf_manager_);
  const auto &new_actions = astar.GetActions(0, false, robot.pos,
                                             robot.mission.internal_mission.to);
  AppendToVector(new_actions, rtn);
  UpdateSafeIntervalsWithActions(0, robot.pos, *rtn);
}

void SippSolver::PlanWmsMission(const RobotInfo &robot, ActionWithTimeSeq *rtn) {
  // Plan for the first half.
  int second_half_start_time_ms = 0;
  RobotInfo tmp_robot = robot;
  if (!robot.shelf_attached) {
    SippAstar astar(map_, safe_intervals_, shelf_manager_);
    ActionWithTimeSeq new_actions = astar.GetActions(0, false, robot.pos,
                                               robot.mission.wms_mission.pick_from.loc);
    if (!new_actions.empty()) {
      second_half_start_time_ms = new_actions.back().end_time_ms;
    }
    new_actions.push_back({Action::ATTACH, second_half_start_time_ms,
                           second_half_start_time_ms + GetActionCostInTime(Action::ATTACH)});
    AppendToVector(new_actions, rtn);

    for (const ActionWithTime &e : new_actions) {
      ApplyActionOnRobot(e.action, &tmp_robot);
    }
  }

  // Plan for the second half.
  SippAstar astar(map_, safe_intervals_, shelf_manager_);
  const auto &new_actions = astar.GetActions(second_half_start_time_ms,
                                             true,
                                             tmp_robot.pos,
                                             robot.mission.wms_mission.drop_to.loc);
  AppendToVector(new_actions, rtn);
  UpdateSafeIntervalsWithActions(0, robot.pos, *rtn);
}

void SippSolver::UpdateSafeIntervalsWithActions(int start_time_ms, Position pos, const ActionWithTimeSeq &seq) {
  for (ActionWithTime awt : seq) {
//    cout << "position: " << pos.to_string() << " action: " << awt.to_string() << endl;
    if (awt.action != Action::MOVE) {
      pos = ApplyActionOnPosition(awt.action, pos);
      continue;
    }

    int prev_interval_end_ms = awt.start_time_ms + kBufferDurationMs;
    safe_intervals_.at(pos.loc).RemoveInterval(start_time_ms, prev_interval_end_ms);
    pos = ApplyActionOnPosition(awt.action, pos);
    start_time_ms = awt.end_time_ms - kBufferDurationMs;
  }

//  cout << "last position: " << pos.to_string() << endl;
  safe_intervals_.at(pos.loc).RemoveInterval(start_time_ms);
}

}