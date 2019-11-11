#ifndef KIVA_SIMULATION_SRC_KS_SCHEDULER_H_
#define KIVA_SIMULATION_SRC_KS_SCHEDULER_H_

#include <set>
#include <mutex>
#include <queue>

#include "common_types.h"
#include "ks_robotmanager.h"
#include "ks_actiongraph.h"
#include "ks_map.h"
#include "path_finder/sipp_solver.h"
#include "ks_simulator.h"
#include "interface/wms_scheduler_types.h"
#include "interface/scheduler_simulator_types.h"
#include "ks_scheduler_common.h"

namespace ks {
class KsWms;

class KsScheduler {
 public:
  KsScheduler(const KsMap& ks_map) : map_(ks_map), robot_manager_(ks_map) {};
  void Init(KsWms *wms_p, KsSimulator *simulator_p);

  // Thread 1, handle mission assignments, replan and generate the action dependency graph.
  void Run();

  // Thread 2.
  // 1. Handle updates from robots, modify action dependency graph and notify wms on mission status.
  // 2. Send commands to robots.
  void AdgRunner();

  void AddMission(WmsMission mission);
  void ReportActionDone(CommandReport r);

  const std::vector<RobotInfo>& GetRobotInfo() { return robot_manager_.GetRobotInfo(); };

 private:

  const KsMap& map_;
  KsWms* wms_p_;
  KsSimulator* simulator_p_;

  // This structure is only accessed by thread 2, used as a cache, so no need to lock.
  std::vector<MissionReport> mq_to_wms_;

  std::mutex mutex_io_up_;
  std::set<WmsMission> missions_to_assign_;

  std::mutex mutex_io_down_;
  // Contains actions that robots had finished.
  std::queue<CommandReport> robot_report_;

  std::mutex mutex_;
  // Maintain robot status and the action dependency graph, the two data structures below should
  // be protected by mutex_ and kept in sync.
  KsRobotManager robot_manager_;
  KsActionGraph action_graph_;
  SippSolver* sipp_p_;
};

}

#endif