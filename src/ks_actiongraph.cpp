#include "ks_actiongraph.h"

namespace ks {

using namespace std;

std::map<int, std::vector<Action>> KsActionGraph::GetCutInternal() {
  return std::map<int, std::vector<Action>>();
}

void KsActionGraph::SetPlan(const std::vector<ActionSeqWithTime>& action_seq) {

  return;
}

std::vector<Action> KsActionGraph::UpdateRobotStatus(int robot_id, Action a) {

  return {};
}

}
