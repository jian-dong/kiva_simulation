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
  int start_ms;
  int end_ms;
  int robot_id;

  Interval(int start_ms, int end_ms, int robot_id) :
      start_ms(start_ms), end_ms(end_ms), robot_id(robot_id) {
    assert(start_ms < end_ms);
  };

  Interval(int start_ms, int end_ms) : start_ms(start_ms), end_ms(end_ms) {
    assert(start_ms < end_ms);
    robot_id = -1;
  };

  Interval &operator=(const Interval &o) = default;

  bool Includes(int time_ms) const {
    return start_ms <= time_ms
        && time_ms < end_ms;
  }

  bool DoesNotIntersect(int o_start_ms, int o_end_ms) const {
    if (o_end_ms <= start_ms) {
      return true;
    }
    if (o_start_ms >= end_ms) {
      return true;
    }
    return false;
  }

  int Length() {
    return end_ms - start_ms;
  }

  bool operator<(const Interval &o) const {
    return start_ms < o.start_ms;
  }

  std::string to_string() {
    return std::to_string(start_ms) + "->"
        + (end_ms >= (kIntInf / 2) ? "INF" : std::to_string(end_ms))
        + " by robot " + std::to_string(robot_id);
  }
};

class IntervalSet {
 public:
  IntervalSet() = default;
  std::vector<Interval> intervals_;

  void AddInterval(int start_ms, int robot_id) {
    AddInterval(start_ms, kIntInf, robot_id);
  }

  void AddInterval(int start_ms, int end_ms, int robot_id) {
    intervals_.emplace_back(start_ms, end_ms, robot_id);
    std::sort(intervals_.begin(), intervals_.end());
    SanityCheck();
  }

  void SanityCheck() const {
    for (int i = 0; i < ((int) intervals_.size() - 1); i++) {
      if (intervals_[i].end_ms > intervals_[i + 1].start_ms) {
        LogFatal("Invalid interval.");
      }
    }
  }
};

class IntervalSeq {
 public:
  std::vector<Interval> intervals_;
  std::vector<Interval> unsafe_intervals_;
  IntervalSeq() {
    intervals_.clear();
    intervals_.emplace_back(0, kIntInf);
    unsafe_intervals_.clear();
  };

  IntervalSeq(const IntervalSeq &o) {
    intervals_ = o.intervals_;
    unsafe_intervals_ = o.unsafe_intervals_;
  };

  void RemoveInterval(int start_ms, int end_ms, int robot_id) {
//    std::cout << "removal: start: " << std::to_string(start) << " end: " << std::to_string(end) << std::endl;
    int index = GetIntervalIndex(start_ms);
    Interval tmp = intervals_[index];
    assert(tmp.start_ms <= start_ms && end_ms <= tmp.end_ms);
    intervals_.erase(intervals_.begin() + index);
    if (tmp.start_ms < start_ms) {
      Interval tmp_1 = Interval(tmp.start_ms, start_ms);
      intervals_.push_back(tmp_1);
    }

    if (end_ms < tmp.end_ms) {
      Interval tmp_2 = Interval(end_ms, tmp.end_ms);
      intervals_.push_back(tmp_2);
    }

    std::sort(intervals_.begin(), intervals_.end());

    unsafe_intervals_.emplace_back(start_ms, end_ms, robot_id);
    std::sort(unsafe_intervals_.begin(), unsafe_intervals_.end());

    SanityCheck();
  }

  void RemoveInterval(int start_ms, int robot_id) {
//    std::cout << "removal: start: " << std::to_string(start) << std::endl;
    int index = GetIntervalIndex(start_ms);
    Interval tmp = intervals_[index];
    assert(tmp.end_ms == kIntInf);
    intervals_.erase(intervals_.begin() + index, intervals_.end());
    if (tmp.start_ms < start_ms) {
      Interval last = Interval(tmp.start_ms, start_ms);
      intervals_.push_back(last);
    }
    unsafe_intervals_.emplace_back(start_ms, kIntInf, robot_id);
    SanityCheck();
  }

  [[nodiscard]] int GetIntervalIndex(int time_ms) const {
    for (int i = 0; i < (int) intervals_.size(); i++) {
      if (intervals_[i].Includes(time_ms)) {
        return i;
      }
    }

    std::cout << "Cannot find interval: " << to_string() << " time: " << time_ms << std::endl;
    LogFatal("Cannot find interval.");
    exit(0);
  }

  void Clear(int robot_id) {
    intervals_.clear();
    unsafe_intervals_.emplace_back(0, kIntInf, robot_id);
  }

  [[nodiscard]] int Size() const {
    return intervals_.size();
  }

  [[nodiscard]] Interval Get(int i) const {
    return intervals_.at(i);
  }

  [[nodiscard]] std::string to_string() const {
    std::string rtn;
    for (Interval interval : intervals_) {
      rtn += interval.to_string() + ";";
    }
    rtn += "unsafe:";
    for (Interval interval : unsafe_intervals_) {
      rtn += interval.to_string() + ";";
    }
    return rtn;
  }

  bool Unused() {
    if (intervals_.size() == 1 && intervals_[0].start_ms == 0 && intervals_[0].end_ms == kIntInf) {
      return true;
    } else {
      return false;
    }
  }

  bool AllUsed() {
    return intervals_.empty();
  }

  void Reset() {
    intervals_.clear();
    intervals_.emplace_back(0, kIntInf);
    unsafe_intervals_.clear();
  }

 private:
  void SanityCheck() const {
    for (int i = 0; i < ((int) intervals_.size() - 1); i++) {
      if (intervals_[i].end_ms > intervals_[i + 1].start_ms) {
        LogFatal("Invalid interval." + to_string());
      }
    }
  }
};

struct PfRequest {
  // Includes all robots.
  std::vector<RobotInfo> robots;
  std::vector<ActionWithTimeSeq> prev_plan;
  int start_time_ms;
};

struct PfResponse {
  // Includes all robots. Empty for no action(wait).
  std::vector<ActionWithTimeSeq> plan;
};

class SippSolver {
 public:
  SippSolver(const KsMap &ks_map)
      : map_(ks_map) {};
  PfResponse FindPath(const PfRequest &req, ShelfManager *shelf_manager_p);

 private:
  void PlanInternalMission(const int start_time_ms, const RobotInfo &robot, ActionWithTimeSeq *rtn);
  void PlanWmsMission(const int start_time_ms, const RobotInfo &robot, ActionWithTimeSeq *rtn);
  void UpdateSafeIntervalsWithActions(int start_time_ms,
                                      int cur_time_ms,
                                      Position pos,
                                      const ActionWithTimeSeq &seq,
                                      int robot_id);

  const KsMap &map_;
  std::map<Location, IntervalSeq> safe_intervals_;
  int robot_count_;
  ShelfManager* shelf_manager_p_;
};

}

#endif //KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_H_
