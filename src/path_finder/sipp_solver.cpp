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

PfResponse SippSolver::FindPath(const PfRequest &req, ShelfManager *shelf_manager_p) {
//  cout << "called find path" << endl;
  // 1. Initialize all safe intervals, for robots with no mission, the location they stay is always non-safe.
  shelf_manager_p_ = shelf_manager_p;
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
    cout << "Planning for robot: " << robot_id << endl;
    const RobotInfo &robot = req.robots[robot_id];
    if (!robot.has_mission) {
      cout << "robot has no mission" << endl;
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
  cout << "from: " << robot.pos.loc.to_string() << " to: " << robot.mission.internal_mission.to.to_string();
  SippAstar astar(map_, safe_intervals_, shelf_manager_p_);
  const auto &new_actions = astar.GetActions(0, false, robot.pos,
                                             robot.mission.internal_mission.to);
  AppendToVector(new_actions, rtn);
  UpdateSafeIntervalsWithActions(0, robot.pos, *rtn);
}

void SippSolver::PlanWmsMission(const RobotInfo &robot, ActionWithTimeSeq *rtn) {
  cout << "from: " << robot.pos.loc.to_string()
      << " via: " << robot.mission.wms_mission.pick_from.loc.to_string()
      << " to: " << robot.mission.wms_mission.drop_to.loc.to_string()
      << endl;
  // Plan for the first half.
  int end_time = 0;
  Position end_pos = robot.pos;

  if (!robot.shelf_attached) {
    SippAstar astar(map_, safe_intervals_, shelf_manager_p_);
    ActionWithTimeSeq new_actions = astar.GetActions(0, false, robot.pos,
                                                     robot.mission.wms_mission.pick_from.loc);
    if (!new_actions.empty()) {
      end_time = new_actions.back().end_time_ms;
      end_pos = new_actions.back().end_pos;
    }

    new_actions.push_back({Action::ATTACH,
                           end_time,
                           end_time + GetActionCostInTime(Action::ATTACH),
                           end_pos,
                           end_pos});
    end_time = end_time + GetActionCostInTime(Action::ATTACH);

    AppendToVector(new_actions, rtn);
    shelf_manager_p_->RemoveMapping(robot.mission.wms_mission.shelf_id,
                                    robot.mission.wms_mission.pick_from.loc);
  }

  // Plan for the second half.
  SippAstar astar(map_, safe_intervals_, shelf_manager_p_);
  ActionWithTimeSeq new_actions_1 = astar.GetActions(end_time,
                                                     true,
                                                     end_pos,
                                                     robot.mission.wms_mission.drop_to.loc);

  end_time = new_actions_1.back().end_time_ms;
  end_pos = new_actions_1.back().end_pos;

  new_actions_1.push_back({Action::DETACH,
                           end_time,
                           end_time + GetActionCostInTime(Action::DETACH),
                           end_pos,
                           end_pos});
  shelf_manager_p_->AddMapping(robot.mission.wms_mission.shelf_id,
                               robot.mission.wms_mission.drop_to.loc);

  AppendToVector(new_actions_1, rtn);
  UpdateSafeIntervalsWithActions(0, robot.pos, *rtn);

//  cout << "Finished update safe intervals" << endl;
}

void SippSolver::UpdateSafeIntervalsWithActions(int start_time_ms, Position pos, const ActionWithTimeSeq &seq) {
  for (ActionWithTime awt : seq) {
//    cout << "position: " << pos.to_string() << " action: " << awt.to_string() << endl;
    if (awt.action != Action::MOVE) {
      pos = ApplyActionOnPosition(awt.action, pos);
      assert(pos == awt.end_pos);
      continue;
    }

    int prev_interval_end_ms = awt.start_time_ms + kBufferDurationMs;
    safe_intervals_.at(pos.loc).RemoveInterval(start_time_ms, prev_interval_end_ms);
    pos = ApplyActionOnPosition(awt.action, pos);
    start_time_ms = awt.end_time_ms;
  }

//  cout << "last position: " << pos.to_string() << endl;
  safe_intervals_.at(pos.loc).RemoveInterval(start_time_ms);
}

}