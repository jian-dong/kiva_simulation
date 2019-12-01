#include "sipp_astar.h"

#include <set>

#include "ks_scheduler_common.h"

namespace ks {

namespace sipp_astar {

using namespace std;

ActionWithTimeSeq SippAstar::GetActions(int start_time_ms,
                                        bool has_shelf,
                                        Position pos,
                                        Location dest) {
  has_shelf_ = has_shelf;
  dest_ = dest;
  int start_safe_interval_index = safe_intervals_.at(pos.loc).GetIntervalIndex(start_time_ms);
  src_ = {pos, start_time_ms, start_safe_interval_index};
  State start(pos, start_time_ms, start_safe_interval_index, 0, GetHeuristicMs(pos.loc, dest_));
  open_.push(start);

  State cur_state;

//  cout << "from: " << pos.to_string() << " to: " << dest_.to_string() << endl;

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
    for (const pair<State, ActionWithTime> &successor : successors) {
      const State &new_state = successor.first;
      const ActionWithTime &action_to_new_state = successor.second;

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

      prev_[new_state.stp] = {action_to_new_state, cur_state.stp};
      open_.push(new_state);
    }
  }

  LogFatal("Cannof find a path. from: " + pos.to_string() + " to: " + dest.to_string()
               + " with shelf: " + to_string(has_shelf));
  exit(0);
}

int SippAstar::GetHeuristicMs(Location a, Location b) {
  // TODO: add turning cost into consideration.
  // This program assumes the moving speed is 1 meter per second.
  return GetManhattanDist(a, b) * kMillisecondsPerSecond;
}

std::vector<std::pair<State, ActionWithTime>> SippAstar::GenSuccessors(const State &cur_state) {
  vector<pair<State, ActionWithTime>> rtn;
  for (Action a : kSippActions) {
    Position new_pos = cur_state.stp.pos;
    ApplyActionOnPosition(a, &new_pos);
    if (!map_.IsLocationPassable(new_pos.loc)) {
      continue;
    }
    if (has_shelf_ && shelf_manager_p_->HasShelf(new_pos.loc)) {
      continue;
    }
    int action_duration = GetActionCostInTime(a);
    int arrival_time_start = cur_state.stp.time_ms + action_duration;
    // May change the way to calculate end time to add a safe interval.
    int arrival_time_end = GetSafeInterval(cur_state.stp).end_ms - kBufferDurationMs
        + action_duration;
    assert(arrival_time_start <= arrival_time_end);
    for (int i = 0; i < safe_intervals_.at(new_pos.loc).Size(); i++) {
      const Interval &interval = safe_intervals_.at(new_pos.loc).Get(i);
      if (interval.DoesNotIntersect(arrival_time_start, arrival_time_end)) {
        continue;
      }
      int arrival_time = max(arrival_time_start, interval.start_ms);
      if (arrival_time + kBufferDurationMs > interval.end_ms) {
        continue;
      }

      State new_state(new_pos, arrival_time, i, arrival_time, GetHeuristicMs(new_pos.loc, dest_));
      rtn.emplace_back(new_state,
                              ActionWithTime(a,
                                             arrival_time - action_duration, arrival_time,
                                             cur_state.stp.pos,
                                             new_pos));
    }
  }
  return rtn;
}

ActionWithTimeSeq SippAstar::GenActionSeq(State cur_state) {
  ActionWithTimeSeq rtn;
  SpatioTemporalPoint stp = cur_state.stp;
  while (stp.NotEqualsTo(src_)) {
    const auto &prev = prev_[stp];
//    cout << "pushed action with time: " << prev.action_with_time.to_string() << endl;
    rtn.push_back(prev.action_with_time);
    stp = prev.stp;
  }
  std::reverse(rtn.begin(), rtn.end());
  return rtn;
}

Interval SippAstar::GetSafeInterval(SpatioTemporalPoint stp) {
  return safe_intervals_.at(stp.pos.loc).Get(stp.safe_interval_index);
}

}
}