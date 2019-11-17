#ifndef KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_H_
#define KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_H_

#include <set>
#include <vector>
#include <list>
#include <algorithm>
#include <iostream>

#include "common_types.h"
#include "ks_map.h"
#include "ks_scheduler_common.h"
#include "sipp_common.h"

namespace ks {

class ShelfManager;

struct Interval {
  double start;
  double end;
  Interval(double start, double end) : start(start), end(end) {};

  bool Includes(double time) const {
    return time >= start && time <= end;
  }

  bool Intersects(double o_start, double o_end) const {
    return (start < o_start && o_start < end) || (start < o_end && o_end < end);
  }

  double Length() {
    assert(end >= start);
    return end - start;
  }

  bool operator<(const Interval &o) const {
    return start < o.start;
  }

  std::string to_string() {
    return std::to_string(start) + "->" + (end >= kDoubleInf ? "INF" : std::to_string(end));
  }
};

class IntervalSeq {
 public:
  std::vector<Interval> intervals_;
  IntervalSeq() {
    intervals_.clear();
    intervals_.emplace_back(0, kDoubleInf);
  };

  IntervalSeq(const IntervalSeq &o) {
    intervals_ = o.intervals_;
  };

  void RemoveInterval(double start, double end) {
    std::cout << "removal: start: " << std::to_string(start) << " end: " << std::to_string(end) << std::endl;
    int index = GetIntervalIndex(start);
    Interval tmp = intervals_[index];
    intervals_.erase(intervals_.begin() + index);
    Interval tmp_1 = Interval(tmp.start, start);
    Interval tmp_2 = Interval(end, tmp.end);
    if (tmp_1.Length() > 2 * kBufferDuration) {
      intervals_.push_back(tmp_1);
    }
    if (tmp_2.Length() > 2 * kBufferDuration) {
      intervals_.push_back(tmp_2);
    }
    std::sort(intervals_.begin(), intervals_.end());
    SanityCheck();
  }

  void RemoveInterval(double start) {
    std::cout << "removal: start: " << std::to_string(start) << std::endl;
    int index = GetIntervalIndex(start);
    Interval tmp = intervals_[index];
    intervals_.erase(intervals_.begin() + index, intervals_.end());

    Interval last = Interval(tmp.start, start);
    if (last.Length() > 2 * kBufferDuration) {
      intervals_.push_back(last);
    }
    SanityCheck();
  }

  int GetIntervalIndex(double time) {
    for (int i = 0; i < intervals_.size(); i++) {
      if (intervals_[i].Includes(time)) {
        return i;
      }
    }
    LogFatal("Cannot find interval.");
  }

  void Clear() {
    intervals_.clear();
  }

  int Size() const {
    return intervals_.size();
  }

  Interval Get(int i) const {
    return intervals_.at(i);
  }

  std::string to_string() const {
    std::string rtn;
    for (Interval interval : intervals_) {
      rtn += interval.to_string() + ";";
    }
    return rtn;
  }

 private:
  void SanityCheck() const {
    for (int i = 0; i < ((int) intervals_.size() - 1); i++) {
      if (intervals_[i].end > intervals_[i + 1].start) {
        LogFatal("Invalid interval." + to_string());
      }
    }
  }
};

struct PfRequest {
  // Includes all robots.
  std::vector<RobotInfo> robots;
};

struct PfResponse {
  // Includes all robots. Empty for no action(wait).
  std::vector<ActionWithTimeSeq> plan;
};

class SippSolver {
 public:
  SippSolver(const KsMap &ks_map, ShelfManager* shelf_manager)
      : map_(ks_map), shelf_manager_(shelf_manager) {};
  PfResponse FindPath(const PfRequest &req);

 private:
  void PlanInternalMission(const RobotInfo &robot, ActionWithTimeSeq *rtn);
  void PlanWmsMission(const RobotInfo &robot, ActionWithTimeSeq *rtn);
  void UpdateSafeIntervalsWithActions(double start_time, Position pos, const ActionWithTimeSeq &seq);

  const KsMap &map_;
  ShelfManager* shelf_manager_;
  std::map<Location, IntervalSeq> safe_intervals_;
  int robot_count_;
};

}

#endif //KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_H_
