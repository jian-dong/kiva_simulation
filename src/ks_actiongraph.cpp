#include "ks_actiongraph.h"

#include <algorithm>

#include "logger.h"

namespace ks {

using namespace std;

namespace {
set<int> GetRobotWithActions(const std::vector<ActionWithTimeSeq> &plan) {
  set<int> rtn;
  int robot_count = plan.size();
  for (int i = 0; i < robot_count; i++) {
    if (!plan[i].empty()) {
      rtn.insert(i);
    }
  }
  return rtn;
}
}

void KsActionGraph::Cut(vector<RobotInfo> *robot_info,
                        ShelfManager *shelf_manager,
                        vector<ActionWithTimeSeq> *remaining_plan,
                        int &start_time_ms) {
  cut_started_ = true;
  start_time_ms = 0;
  for (int i = 0; i < robot_count_; i++) {
    if (to_send_action_index_[i] > 0) {
      start_time_ms = max(start_time_ms, plan_[i][to_send_action_index_[i] - 1].end_time_ms);
    }
  }
  *remaining_plan = plan_;

  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    for (int j = replied_action_index_[robot_id] + 1; j < (int) to_send_action_index_[robot_id]; j++) {
      Action a = plan_[robot_id][j].action;
      ApplyActionOnRobot(a, &((*robot_info)[robot_id]), shelf_manager);
    }
  }
}

void KsActionGraph::SetPlan(const std::vector<ActionWithTimeSeq> &new_plan, const std::set<Edge> &edges) {
  for (int i = 0; i < robot_count_; i++) {
    if (!new_plan[i].empty()) {
      assert(plan_[i].empty());
      plan_[i] = new_plan[i];
    }
  }

  for (const auto& e : edges) {
//    assert(e.from.action_index < (int) plan_[e.from.robot_id].size());
    assert(e.to.action_index < (int)plan_[e.to.robot_id].size());
    if (e.from.action_index <= replied_action_index_[e.from.robot_id]) {
      continue;
    }
    if (e.from.action_index >= (int) plan_[e.from.robot_id].size()) {
      continue;
    }
//    cout << "newly added edges: " << e.to_string() << endl;
    adj_.AddEdge(e.from, e.to);
  }
  cut_started_ = false;
}

void KsActionGraph::UpdateRobotStatus(int robot_id, Action a) {
  assert(plan_[robot_id][replied_action_index_[robot_id] + 1].action == a);
  replied_action_index_[robot_id]++;
  assert(replied_action_index_[robot_id] < to_send_action_index_[robot_id]);
  adj_.RemoveAllEdgesFrom({robot_id, replied_action_index_[robot_id]});

  if (replied_action_index_[robot_id] == (((int) plan_[robot_id].size()) - 1)) {
    // Mission finished.
    plan_[robot_id].clear();
    replied_action_index_[robot_id] = -1;
    to_send_action_index_[robot_id] = 0;
  }
}

vector<vector<Action>> KsActionGraph::GetCommands(const std::vector<RobotInfo> &robot_info, int &acked) {
  vector<vector<Action>> rtn(robot_count_);

  if (cut_started_) {
    return rtn;
  }

  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    int action_index;
    for (action_index = to_send_action_index_[robot_id]; action_index < (int) plan_[robot_id].size(); action_index++) {
      if (action_index - replied_action_index_[robot_id] > kCommandsToSent) {
        break;
      }
      Node to_send(robot_id, action_index);
      if (adj_.CanSendAction(to_send)) {
        rtn[robot_id].push_back(plan_[robot_id][action_index].action);
      } else {
        break;
      }
    }
    to_send_action_index_[robot_id] = action_index;
  }
  return rtn;
}

ActionPlan KsActionGraph::GetCurrentPlan() const {
  ActionPlan rtn(robot_count_);

  for (int i = 0; i < robot_count_; i++) {
    for (int j = replied_action_index_[i] + 1; j < ((int) plan_[i].size()); j++) {
      rtn[i].push_back(plan_[i][j]);
    }
  }
  return rtn;
}

std::set<Edge> GetNewDependency(const std::vector<ActionWithTimeSeq> &plan,
                                const std::vector<ActionWithTimeSeq> &remaining_plan) {
  set<int> plan_robot_set(GetRobotWithActions(plan));
  set<int> remaining_plan_robot_set(GetRobotWithActions(remaining_plan));

  set<Edge> rtn;

  const auto& d0 = GetDependencyWithinPlan(plan_robot_set, plan);
  rtn.insert(d0.begin(), d0.end());

  const auto& d1 = GetDependencyInterPlan(plan_robot_set, plan, remaining_plan_robot_set, remaining_plan);
  rtn.insert(d1.begin(), d1.end());

  const auto& d2 = GetDependencyInterPlan(remaining_plan_robot_set, remaining_plan, plan_robot_set, plan);
  rtn.insert(d2.begin(), d2.end());

  return rtn;
}

std::set<Edge> GetDependencyWithinPlan(const std::set<int> &robots, const std::vector<ActionWithTimeSeq> &plan) {
  set<Edge> rtn;
  for (int robot_0 : robots) {
    for (int action_index_0 = 0; action_index_0 < (int) plan[robot_0].size(); action_index_0++) {
      for (int robot_1 : robots) {
        if (robot_0 == robot_1) {
          continue;
        }
        for (int action_index_1 = 0; action_index_1 < (int) plan[robot_1].size(); action_index_1++) {
          ActionWithTime awt_0 = plan[robot_0][action_index_0];
          ActionWithTime awt_1 = plan[robot_1][action_index_1];
          if (awt_0.start_pos.loc == awt_1.end_pos.loc && awt_0.start_time_ms <= awt_1.start_time_ms) {
            // Assert insertion success.
            assert(rtn.insert(Edge({robot_0, action_index_0}, {robot_1, action_index_1})).second);
            break;
          }
        }
      }
    }
  }
  return rtn;
}

std::set<Edge> GetDependencyInterPlan(const std::set<int> &robots_0,
                                      const std::vector<ActionWithTimeSeq> &plan_0,
                                      const std::set<int> &robots_1,
                                      const std::vector<ActionWithTimeSeq> &plan_1) {
  set<Edge> rtn;
  for (int robot_0 : robots_0) {
    for (int action_index_0 = 0; action_index_0 < (int) plan_0[robot_0].size(); action_index_0++) {
      for (int robot_1 : robots_1) {
        if (robot_0 == robot_1) {
          continue;
        }
        for (int action_index_1 = 0; action_index_1 < (int) plan_1[robot_1].size(); action_index_1++) {
          ActionWithTime awt_0 = plan_0[robot_0][action_index_0];
          ActionWithTime awt_1 = plan_1[robot_1][action_index_1];
          if (awt_0.start_pos.loc == awt_1.end_pos.loc && awt_0.start_time_ms <= awt_1.start_time_ms) {
            // Assert insertion success.
            assert(rtn.insert(Edge({robot_0, action_index_0}, {robot_1, action_index_1})).second);
            break;
          }
        }
      }
    }
  }

  return rtn;
}

std::set<Edge> BuildDependencyOld(const std::vector<ActionWithTimeSeq> &plan) {
  int robot_count = plan.size();
  std::set<Edge> rtn;
  // TODO: are there any better data structures can be applied here? bloom filter?
  // Add type II edges, type I edges(between actions of the same robot) are implicit.
  for (int robot_0 = 0; robot_0 < robot_count; robot_0++) {
    for (int action_index_0 = 0; action_index_0 < (int)plan[robot_0].size(); action_index_0++) {
      for (int robot_1 = 0; robot_1 < robot_count; robot_1++) {
        if (robot_0 == robot_1) {
          continue;
        }
        for (int action_index_1 = 0; action_index_1 < (int)plan[robot_1].size(); action_index_1++) {
          ActionWithTime awt_0 = plan[robot_0][action_index_0];
          ActionWithTime awt_1 = plan[robot_1][action_index_1];
          if (awt_0.start_pos.loc == awt_1.end_pos.loc && awt_0.start_time_ms <= awt_1.start_time_ms) {
            assert(awt_0.start_time_ms < awt_1.end_time_ms);
            assert(rtn.insert(Edge({robot_0, action_index_0}, {robot_1, action_index_1})).second);
            break;
          }
        }
      }
    }
  }
  return rtn;
}


}
