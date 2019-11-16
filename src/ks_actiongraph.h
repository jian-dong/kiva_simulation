#ifndef KIVA_SIMULATION_SRC_KS_ACTIONGRAPH_H_
#define KIVA_SIMULATION_SRC_KS_ACTIONGRAPH_H_

#include <map>
#include <vector>
#include <list>

#include "common_types.h"
#include "ks_scheduler_common.h"

namespace ks {

struct Node {
  int robot_id;
  int action_index;

  Node() = default;
  Node(int robot_id, int action_index) : robot_id(robot_id), action_index(action_index) {};
};

class KsActionGraph {
 public:
  KsActionGraph() {
    global_plan_.resize(kRobotCount);
  };
  std::map<int, std::vector<Action>> GetCutInternal();
  void SetPlan(const std::vector<ActionWithTimeSeq>& global_plan);
  std::vector<Action> UpdateRobotStatus(int robot_id, Action a);
 private:
  std::vector<std::vector<Action>> global_plan_;
  std::map<Node, std::list<Node>> adj_;
};

}

#endif