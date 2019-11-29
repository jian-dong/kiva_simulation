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

void PrintActionSequence(const ActionWithTimeSeq & as) {
  cout << "Actions: ";
  for (ActionWithTime a : as) {
    cout << a.to_string() << " ";
  }
  cout << endl;
}
}

PfResponse SippSolver::FindPath(const PfRequest &req, ShelfManager *shelf_manager_p) {
  // 1. Initialize all safe intervals, for robots with no mission, the location they stay is always non-safe.
  shelf_manager_p_ = shelf_manager_p;
  robot_count_ = req.robots.size();
  safe_intervals_.clear();
  const vector<Location> &passable = map_.GetPassableLocations();
  for (const Location &l : passable) {
    safe_intervals_[l] = IntervalSeq();
  }

  for (int i = 0; i < robot_count_; i++) {
    // If prev_plan is empty, then the whole interval will be removed.
    // If prev_plan is not empty, remove intervals according to the plan.
    UpdateSafeIntervalsWithActions(0, req.robots[i].pos, req.prev_plan[i], i);
  }

  // 2. For each robot, plan a route.
  PfResponse rtn;
  rtn.plan = req.prev_plan;
  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    const RobotInfo &robot = req.robots[robot_id];
    if (!robot.has_mission) {
      continue;
    }
    if (!req.prev_plan[robot_id].empty()) {
      // The robot has unfinished plans.
      continue;
    }
    safe_intervals_[robot.pos.loc] = IntervalSeq();

    if (robot.mission.is_internal) {
      PlanInternalMission(robot, &rtn.plan[robot_id]);
//      PrintActionSequence(rtn.plan[robot_id]);
    } else {
      PlanWmsMission(robot, &rtn.plan[robot_id]);
    }
  }

  return rtn;
}

void SippSolver::PlanInternalMission(const RobotInfo &robot, ActionWithTimeSeq *rtn) {
//  cout << "from: " << robot.pos.loc.to_string() << " to: " << robot.mission.internal_mission.to.to_string();
  SippAstar astar(map_, safe_intervals_, shelf_manager_p_);
  const auto &new_actions = astar.GetActions(0, false, robot.pos,
                                             robot.mission.internal_mission.to);
  AppendToVector(new_actions, rtn);
  UpdateSafeIntervalsWithActions(0, robot.pos, *rtn, robot.id);
}

void SippSolver::PlanWmsMission(const RobotInfo &robot, ActionWithTimeSeq *rtn) {
  int end_time_ms = 0;
  Position end_pos = robot.pos;

  bool shelf_attached = robot.shelf_attached;
  Position from = robot.pos;
  Location to;
  if (shelf_attached) {
    to = robot.mission.wms_mission.drop_to.loc;
  } else {
    to = robot.mission.wms_mission.pick_from.loc;
  }

  SippAstar astar(map_, safe_intervals_, shelf_manager_p_);
  const auto new_actions = astar.GetActions(0, shelf_attached, from, to);
  AppendToVector(new_actions, rtn);

  if (!new_actions.empty()) {
    end_time_ms = new_actions.back().end_time_ms;
    end_pos = new_actions.back().end_pos;
  }

  if (shelf_attached) {
    rtn->push_back({Action::DETACH,
                    end_time_ms,
                    end_time_ms + GetActionCostInTime(Action::DETACH),
                    end_pos,
                    end_pos});
    shelf_manager_p_->AddMapping(robot.mission.wms_mission.shelf_id,
                                 robot.mission.wms_mission.drop_to.loc);
  } else {
    rtn->push_back({Action::ATTACH,
                      end_time_ms,
                      end_time_ms + GetActionCostInTime(Action::ATTACH),
                      end_pos,
                      end_pos});
    shelf_manager_p_->RemoveMapping(robot.mission.wms_mission.shelf_id,
                                    robot.mission.wms_mission.pick_from.loc);
  }
  UpdateSafeIntervalsWithActions(0, robot.pos, *rtn, robot.id);
}

void SippSolver::UpdateSafeIntervalsWithActions(int start_time_ms,
                                                Position pos,
                                                const ActionWithTimeSeq &seq,
                                                int robot_id) {
  for (ActionWithTime awt : seq) {
    if (awt.action != Action::MOVE) {
      ApplyActionOnPosition(awt.action, &pos);
      assert(pos == awt.end_pos);
      continue;
    }

    int prev_interval_end_ms = awt.start_time_ms + kBufferDurationMs;
    safe_intervals_.at(pos.loc).RemoveInterval(start_time_ms, prev_interval_end_ms, robot_id);
    ApplyActionOnPosition(awt.action, &pos);
    start_time_ms = awt.end_time_ms;
  }

  safe_intervals_.at(pos.loc).RemoveInterval(start_time_ms, robot_id);
}

}