#ifndef KIVA_SIMULATION_SRC_KS_ACTIONGRAPH_H_
#define KIVA_SIMULATION_SRC_KS_ACTIONGRAPH_H_

#include <map>
#include <vector>

#include "common_types.h"
#include "ks_scheduler_common.h"

namespace ks {
class KsActionGraph {
 public:
  KsActionGraph() = default;
  std::map<int, std::vector<Action>> GetCutInternal();
  void SetPlan(const std::vector<ActionSeqWithTime>& action_seq);
  std::vector<Action> UpdateRobotStatus(int robot_id, Action a);
 private:

};

}

#endif