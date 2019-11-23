#ifndef KIVA_SIMULATION_SRC_KS_SCHEDULER_H_
#define KIVA_SIMULATION_SRC_KS_SCHEDULER_H_

#include <set>
#include <mutex>
#include <queue>

#include "common_types.h"
#include "interface/ks_api.h"
#include "ks_actiongraph.h"
#include "ks_map.h"
#include "ks_robotmanager.h"
#include "ks_scheduler_common.h"
#include "path_finder/sipp_solver.h"

namespace ks {
class KsScheduler : public KsSchedulerApi {
 public:
  KsScheduler(const KsMap& ks_map) : ks_map_(ks_map), robot_manager_(ks_map),
                                     robot_count_(ks_map.actual_robot_count_),
                                     action_graph_(ks_map.actual_robot_count_) {};
  void Init(KsWmsApi *wms_p, KsSimulatorApi *simulator_p);

  // Thread 1, handle mission assignments, replan and generate the action dependency graph.
  void Run();

  // Thread 2.
  // 1. Handle updates from robots, modify action dependency graph and notify wms on mission status.
  // 2. Send commands to robots.
  void AdgRunner();

  void AddMission(WmsMission mission) override;
  void ReportActionDone(CommandReport r) override;

 private:

  const int robot_count_;
  const KsMap& ks_map_;
  KsWmsApi* wms_p_;
  KsSimulatorApi* simulator_p_;

  std::mutex mutex_io_up_;
  std::set<WmsMission> missions_from_wms_;

  std::mutex mutex_io_down_;
  // Contains actions that robots had finished.
  std::queue<CommandReport> robot_reports_;

  std::mutex mutex_;
  // Maintain robot status and the action dependency graph, the two data structures below should
  // be protected by mutex_ and kept in sync.
  KsRobotManager robot_manager_;
  KsActionGraph action_graph_;
  ShelfManager shelf_manager_;
  SippSolver* sipp_p_;
};
}

#endif