#include "sipp_astar.h"

#include <set>

#include "ks_scheduler_common.h"

namespace ks {
namespace sipp_astar {

using namespace std;

namespace {
double GetManhattanDistance(const Location& a, const Location& b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}
}

ActionWithTimeSeq SippAstar::GetActions(double start_time,
                                        bool has_shelf,
                                        Position pos,
                                        Location dest) {
  has_shelf_ = has_shelf;
  dest_ = dest;
  src_ = {pos, 0, 0};
  State start(pos, start_time, 0, 0, GetHeuristic(pos.loc, dest_));
  open_.push(start);

  State cur_state;

  while (!open_.empty()) {
    cur_state = open_.top();
    open_.pop();

//    cout << "examing state: " + cur_state.stp.to_string() << endl;

    if (cur_state.stp.pos.loc == dest_) {
      return GenActionSeq(cur_state);
    }

    if (closed_.find(cur_state.stp) != closed_.end()) {
      continue;
    }
    closed_.insert(cur_state.stp);

    vector<pair<State, ActionWithTime>> successors = GenSuccessors(cur_state);
    for (const pair<State, ActionWithTime> & successor : successors) {
      const State& new_state = successor.first;
      const ActionWithTime& action_to_new_state = successor.second;

      if (closed_.find(new_state.stp) != closed_.end()) {
        continue;
      }

      int tmp_g_value = new_state.past_cost;
      if (g_value_.find(new_state.stp) != g_value_.end()) {
        if (tmp_g_value >= g_value_.at(new_state.stp)) {
          continue;
        }
      }
      g_value_[new_state.stp] = tmp_g_value;

      prev_[new_state.stp] = {action_to_new_state , cur_state.stp};
      open_.push(new_state);
    }
  }

  return ActionWithTimeSeq();
}

double SippAstar::GetHeuristic(Location a, Location b) {
  // TODO: add turning cost into consideration.
  return GetManhattanDist(a, b);
}

std::vector<std::pair<State, ActionWithTime>> SippAstar::GenSuccessors(const State &cur_state) {
  vector<pair<State, ActionWithTime>> rtn;
  for (Action a : kSippActions) {
    Position new_pos = ApplyActionOnPosition(a, cur_state.stp.pos);
    if (!map_.IsLocationPassable(new_pos.loc)) {
      continue;
    }
    if (has_shelf_ && shelf_manager_->HasShelf(new_pos.loc)) {
      continue;
    }
    double action_duration = GetActionCostInTime(a);
    double arrival_time_start = cur_state.stp.time + action_duration;
    // May change the way to calculate end time to add a safe interval.
    double arrival_time_end = GetSafeInterval(cur_state.stp).end
        + action_duration;
    assert(arrival_time_start < arrival_time_end);
    for (int i = 0; i < safe_intervals_.at(new_pos.loc).Size(); i++) {
      const Interval& interval = safe_intervals_.at(new_pos.loc).Get(i);
      if (!interval.Intersects(arrival_time_start, arrival_time_end)) {
        continue;
      }
//      if (interval.start + 2 * kBufferDuration > interval.end) {
//        continue;
//      }
      double arrival_time = max(arrival_time_start, interval.start + kBufferDuration);
      if (arrival_time + kBufferDuration > interval.end) {
        continue;
      }

      State new_state(new_pos, arrival_time, i, arrival_time, GetHeuristic(new_pos.loc, dest_));
      rtn.push_back(make_pair(new_state, ActionWithTime(a, arrival_time - action_duration, arrival_time)));
    }
  }
  return rtn;
}

ActionWithTimeSeq SippAstar::GenActionSeq(State cur_state) {
  ActionWithTimeSeq rtn;
  SpatioTemporalPoint stp = cur_state.stp;
  while (stp != src_) {
    const auto& prev = prev_[stp];
    rtn.push_back(prev.action_with_time);
    stp = prev.stp;
  }
  std::reverse(rtn.begin(), rtn.end());
  return rtn;
}

Interval SippAstar::GetSafeInterval(SpatioTemporalPoint stp) {
  if (safe_intervals_.find(stp.pos.loc) == safe_intervals_.end()) {
    LogFatal("Invalid interval 2");
  }

  return safe_intervals_.at(stp.pos.loc).Get(stp.safe_interval_index);
}

}
}