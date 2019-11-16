#include "ks_actiongraph.h"

#include <algorithm>

#include "logger.h"

namespace ks {

using namespace std;

std::map<int, std::vector<Action>> KsActionGraph::GetCutInternal() {
  std::map<int, std::vector<Action>> rtn;
  for (int i = 0; i < kRobotCount; i++) {
    if (!global_plan_[i].empty()) {
      int cut_count = min(kActionsTillCut, (int)global_plan_[i].size());
      for (int j = 0; j < cut_count; j++) {
        rtn[i].push_back(global_plan_[i][j]);
      }
      global_plan_[i].erase(global_plan_[i].begin() + cut_count, global_plan_[i].end());
    }
  }
  return rtn;
}

void KsActionGraph::SetPlan(const std::vector<ActionWithTimeSeq>& global_plan) {
  for (int i = 0; i < kRobotCount; i++) {
    for (ActionWithTime a : global_plan[i]) {
      global_plan_[i].push_back(a.action);
    }
  }
}

// The current version send out one action at a time.
std::vector<Action> KsActionGraph::UpdateRobotStatus(int robot_id, Action a) {
  // A sanity check.
  if (global_plan_[robot_id].front() != a) {
    LogFatal("Invalid report.");
  }
  global_plan_[robot_id].erase(global_plan_[robot_id].begin());

  if (!global_plan_[robot_id].empty()) {
    return {global_plan_[robot_id].front()};
  } else {
    return {};
  }
}

}
