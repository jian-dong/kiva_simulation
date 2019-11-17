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
  // Seconds since the start of scheduling.
  double time;
  int safe_interval_index;

  SpatioTemporalPoint() = default;
  SpatioTemporalPoint(Position pos, double time, int safe_interval_index)
      : pos(pos),
        time(time),
        safe_interval_index(safe_interval_index) {};

  SpatioTemporalPoint& operator=(const SpatioTemporalPoint& o) = default;

  bool operator<(const SpatioTemporalPoint& o) const {
    if (pos != o.pos) {
      return pos < o.pos;
    }
    if (!DoubleEquals(time, o.time)) {
      return time < o.time;
    }
    return safe_interval_index < o.safe_interval_index;
  }

  bool operator==(const SpatioTemporalPoint& o) const {
    return pos == o.pos && DoubleEquals(time, o.time) && safe_interval_index == o.safe_interval_index;
  }

  bool operator!=(const SpatioTemporalPoint& o) const {
    return !operator==(o);
  }

  std::string to_string() {
    return pos.to_string() + " " + std::to_string(time) + " " + std::to_string(safe_interval_index);
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

  double past_cost;
  double heuristic;

  State() = default;
  State(Position pos, double time, int safe_interval_index, double past_cost, double heuristic)
      : stp(pos, time, safe_interval_index),
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
      ShelfManager* shelf_manager)
      : map_(ks_map), safe_intervals_(safe_intervals), shelf_manager_(shelf_manager) {};

  // Return a sequence of actions to move the robot from src to dest.
  ActionWithTimeSeq GetActions(double start_time, bool has_shelf, Position pos, Location dest);
 private:
  double GetHeuristic(Location a, Location b);
  std::vector<std::pair<State, ActionWithTime>> GenSuccessors(const State &cur_state);
  ActionWithTimeSeq GenActionSeq(State cur_state);

  Interval GetSafeInterval(SpatioTemporalPoint stp);

  const KsMap &map_;
  ShelfManager* shelf_manager_;
  const std::map<Location, IntervalSeq> &safe_intervals_;

  std::set<SpatioTemporalPoint> closed_;
  std::priority_queue<State> open_;
  std::map<SpatioTemporalPoint, double> g_value_;
  std::map<SpatioTemporalPoint, PrevState> prev_;

  SpatioTemporalPoint src_;
  Location dest_;
  bool has_shelf_;
};

}
}
#endif