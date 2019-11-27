#ifndef KIVA_SIMULATION_SRC_KS_ACTIONGRAPH_H_
#define KIVA_SIMULATION_SRC_KS_ACTIONGRAPH_H_

#include <map>
#include <vector>
#include <list>
#include <iostream>

#include "common_types.h"
#include "ks_scheduler_common.h"

namespace ks {
struct Node {
  int robot_id;
  int action_index;

  Node() = default;
  Node(int robot_id, int action_index) : robot_id(robot_id), action_index(action_index) {};

  bool operator<(const Node& o) const {
    if (robot_id == o.robot_id) {
      return action_index < o.action_index;
    }
    return robot_id < o.robot_id;
  }

  std::string to_string() {
    return "robot id:" + std::to_string(robot_id) + "action index:" + std::to_string(action_index);
  }
};

class TwoWayAdjList {
 public:
  TwoWayAdjList() = default;
  TwoWayAdjList(const TwoWayAdjList& o) = default;
  TwoWayAdjList& operator= (const TwoWayAdjList &t) = delete;

  void AddEdge(const Node& from, const Node& to) {
    assert(adj_[from].find(to) == adj_[from].end());
    assert(radj_[to].find(from) == radj_[to].end());
    adj_[from].insert(to);
    radj_[to].insert(from);
  }

  void RemoveAllEdgesFrom(const Node& from) {
    for (const Node& to : adj_[from]) {
      assert(radj_[to].erase(from) == 1);
    }
    adj_[from].clear();
  }

  // An action can be sent out if it has no incoming edge or the only incoming edge is from
  // a precedent action of the same robot.
  bool CanSendAction(const Node& to) {
    if (radj_[to].size() == 0) {
      return true;
    }
//    if (radj_[to].size() == 1) {
//      assert(to.robot_id == radj_[to].begin()->robot_id);
//      return true;
//    }
    return false;
//    return radj_[to].size() == 1;
  }

  void Clear() {
    adj_.clear();
    radj_.clear();
  }

  // A mapping from source node to destination nodes.
  std::map<Node, std::set<Node>> adj_;
  // A mapping from destination node to source nodes.
  std::map<Node, std::set<Node>> radj_;
};

class GlobalPlan {
 public:
  GlobalPlan(int robot_count) : robot_count_(robot_count){
    scheduled_to_change_ = false;
    plan_.resize(robot_count);
    adj_.Clear();
    to_send_action_index_.resize(robot_count);
    replied_action_index_.resize(robot_count);
    for (int i = 0; i < robot_count_; i++) {
      to_send_action_index_[i] = 0;
      replied_action_index_[i] = -1;
    }
  }

  GlobalPlan(const GlobalPlan& o) = default;

  void Clear() {
    for (int i = 0; i < robot_count_; i++) {
      plan_[i].clear();
    }
    adj_.Clear();
    for (int i = 0; i < robot_count_; i++) {
      to_send_action_index_[i] = 0;
      replied_action_index_[i] = -1;
    }
    scheduled_to_change_ = false;
  }

  bool Finished() {
    for (int i = 0; i < robot_count_; i++) {
      if (replied_action_index_[i] < (((int)plan_[i].size()) - 1)) {
        return false;
      }
    }
    return true;
  }

  void UpdateRobotStatus(int robot_id, Action a);
  std::vector<Action> GetActionToSend(int robot_id);
  // This function is idempotent, the current behavior is stop sending new commands, just wait
  // for all the already sent command to finish.
  void Cut(std::vector<RobotInfo> &robot_info, ShelfManager &shelf_manager);
  // This function may be called on the next plan multiple times.
  void SetPlan(const std::vector<ActionWithTimeSeq> &plan);

  const int robot_count_;
  // Initialized to 0, when this value is equal to the action count, all the actions are sent.
  std::vector<int> to_send_action_index_;
  // Initialized to -1, when this is equal to <last_action_index_>, this plan is considered done.
  std::vector<int> replied_action_index_;

  bool scheduled_to_change_;
  std::vector<RobotInfo> cached_robot_info_;
  ShelfManager cached_shelf_manager_;

  std::vector<std::vector<Action>> plan_;
  TwoWayAdjList adj_;
};

class KsActionGraph {
 public:
  KsActionGraph(int robot_count) : robot_count_(robot_count) {
    cur_plan_p_ = new GlobalPlan(robot_count);
    next_plan_p_ = new GlobalPlan(robot_count);
    cur_plan_p_->Clear();
    next_plan_p_->Clear();
  };

  void Cut(std::vector<RobotInfo> &robot_info, ShelfManager &shelf_manager);
  void SetPlan(const std::vector<ActionWithTimeSeq> &plan);
  void UpdateRobotStatus(int robot_id, Action a);
  std::vector<std::vector<Action>> GetCommands();

 private:
  const int robot_count_;
  GlobalPlan* cur_plan_p_;
  GlobalPlan* next_plan_p_;
};
}

#endif