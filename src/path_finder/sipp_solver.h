#ifndef KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_H_
#define KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_H_

#include <set>
#include <vector>
#include <list>

#include "common_types.h"
#include "ks_map.h"
#include "ks_scheduler_common.h"

namespace ks {

struct SafeInterval {
  // Safe interval is inclusive.
  double start;
  double end;
  SafeInterval(double start, double end) : start(start), end(end) {};
};

struct State {
  Location loc;
  Direction dir;
  // Seconds since the start of scheduling.
  double time;
  SafeInterval cur_interval;
};

struct PfRequest {
  // Includes all robots.
  std::vector<RobotInfo> robots;
};

struct PfResponse {
  // Includes all robots. Empty for no action(wait).
  std::vector<ActionSeqWithTime> actions;
};

class SippSolver {
 public:
  SippSolver(const KsMap& ks_map) : map_(ks_map) {};
  PfResponse FindPath(const PfRequest &req);
 private:
  void PlanInternalMission(const RobotInfo& robot);
  void PlanWmsMission(const RobotInfo& robot);

  ActionSeqWithTime GetActions(double start_time, bool has_shelf, Location src, Location target);

  const KsMap& map_;
  std::map<Location, std::list<SafeInterval>> safe_intervals_;
};

}

#endif //KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_H_
