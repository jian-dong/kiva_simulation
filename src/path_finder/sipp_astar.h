#ifndef KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_ASTAR_H_
#define KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_ASTAR_H_

#include <queue>

#include "ks_scheduler_common.h"
#include "ks_map.h"
#include "sipp_common.h"
#include "sipp_solver.h"
#include "utilities.h"

namespace ks {
namespace sipp_astar {

struct SpatioTemporalPoint {
  Position pos;
  // Milliseconds since the start of scheduling.
  int time_ms;
  int safe_interval_index;

  SpatioTemporalPoint() = default;
  SpatioTemporalPoint(Position pos, int time_ms, int safe_interval_index)
      : pos(pos),
        time_ms(time_ms),
        safe_interval_index(safe_interval_index) {};

  SpatioTemporalPoint &operator=(const SpatioTemporalPoint &o) = default;

  bool operator<(const SpatioTemporalPoint &o) const {
    if (pos != o.pos) {
      return pos < o.pos;
    }
    return safe_interval_index < o.safe_interval_index;
  }

  // Intentionally avoids overload the "==" and "!=" operator.
  // Because operator< does not consider consider time_ms while
  // here it is considered. TODO: think of if time_ms can be
  // ignored here.
  bool EqualsTo(const SpatioTemporalPoint &o) const {
    return pos == o.pos
        && time_ms == o.time_ms
        && safe_interval_index == o.safe_interval_index;
  }

  bool NotEqualsTo(const SpatioTemporalPoint &o) const {
    return !EqualsTo(o);
  }

  std::string to_string() {
    return pos.to_string() + " " + std::to_string(time_ms) + " " + std::to_string(safe_interval_index);
  }
};

struct PrevState {
  ActionWithTime action_with_time;
  SpatioTemporalPoint stp;

  PrevState() = default;
  PrevState(ActionWithTime awt, SpatioTemporalPoint stp) : action_with_time(awt), stp(stp) {};
};

struct State {
  SpatioTemporalPoint stp;
  // Past cost and heuristic are both in milliseconds.
  int past_cost;
  int heuristic;

  State() = default;
  State(Position pos, int time_ms, int safe_interval_index, int past_cost, int heuristic)
      : stp(pos, time_ms, safe_interval_index),
        past_cost(past_cost),
        heuristic(heuristic) {};

  bool operator<(const State &o) const {
    if (past_cost + heuristic != o.past_cost + o.heuristic) {
      return past_cost + heuristic > o.past_cost + o.heuristic;
    }
    return heuristic > o.heuristic;
  }
};

class SippAstar {
 public:
  SippAstar(const KsMap &ks_map,
            const std::map<Location, IntervalSeq> &safe_intervals,
            const ShelfManager *shelf_manager_p)
      : map_(ks_map), safe_intervals_(safe_intervals), shelf_manager_p_(shelf_manager_p) {
  };

  // Return a sequence of actions to move the robot from src to dest.
  ActionWithTimeSeq GetActions(int start_time_ms, const bool has_shelf, const Position pos, const Location dest);
 private:
  // Return the heuristic(in milliseconds) from source to destination.
  int GetHeuristicMs(Location a, Location b);
  std::vector<std::pair<State, ActionWithTime>> GenSuccessors(const State &cur_state);
  ActionWithTimeSeq GenActionSeq(State cur_state);

  Interval GetSafeInterval(SpatioTemporalPoint stp);

  const KsMap &map_;
  const ShelfManager *shelf_manager_p_;
  const std::map<Location, IntervalSeq> &safe_intervals_;

  std::set<SpatioTemporalPoint> closed_;
  std::priority_queue<State> open_;
  std::map<SpatioTemporalPoint, int> g_value_;
  std::map<SpatioTemporalPoint, PrevState> prev_;

  SpatioTemporalPoint src_;
  Location dest_;
  bool has_shelf_;
};

}
}
#endif