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

  bool operator<(const Node &o) const {
    if (robot_id == o.robot_id) {
      return action_index < o.action_index;
    }
    return robot_id < o.robot_id;
  }

  std::string to_string() {
    return "robot id:" + std::to_string(robot_id) + "action index:" + std::to_string(action_index);
  }
};

struct Edge {
  Node from, to;
  Edge(Node from, Node to) : from(from), to(to) {};

  bool operator<(const Edge &o) const {
    if ((!(from < o.from)) && (!(o.from < from))) {
      return to < o.to;
    } else {
      return from < o.from;
    }
  }

  std::string to_string() const{
    return std::to_string(from.robot_id) + " " + std::to_string(from.action_index)
        + " : " + std::to_string(to.robot_id) + " " + std::to_string(to.action_index);
  }
};

class TwoWayAdjList {
 public:
  TwoWayAdjList() = default;
  TwoWayAdjList(const TwoWayAdjList &o) = default;
  TwoWayAdjList &operator=(const TwoWayAdjList &t) = default;
  TwoWayAdjList &operator=(TwoWayAdjList &&other) = default;

  void AddEdge(const Node &from, const Node &to) {
    assert(adj_[from].find(to) == adj_[from].end());
    assert(radj_[to].find(from) == radj_[to].end());
    adj_[from].insert(to);
    radj_[to].insert(from);
  }

  std::set<Edge> GetEdgeSet() {
    std::set<Edge> rtn;
    for (const auto & elem : adj_) {
      for (const auto & to: elem.second) {
        rtn.insert({elem.first, to});
      }
    }
    return rtn;
  }

  void AssertContainsEdge(const Node &from, const Node &to) {
    assert(adj_[from].find(to) != adj_[from].end());
    assert(radj_[to].find(from) != radj_[to].end());
  }

  void RemoveAllEdgesFrom(const Node &from) {
    for (const Node &to : adj_[from]) {
//      std::cout << "newly removed edges: " << Edge(from, to).to_string() << std::endl;
      assert(radj_[to].erase(from) == 1);
    }
    adj_[from].clear();
  }

  // An action can be sent out if it has no incoming edge or the only incoming edge is from
  // a precedent action of the same robot.
  bool CanSendAction(const Node &to) {
    return radj_[to].empty();
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

class KsActionGraph {
 public:
  KsActionGraph(int robot_count) : robot_count_(robot_count) {
    plan_.resize(robot_count);
    to_send_action_index_.resize(robot_count);
    replied_action_index_.resize(robot_count);
    for (int i = 0; i < robot_count_; i++) {
      to_send_action_index_[i] = 0;
      replied_action_index_[i] = -1;
    }
    adj_.Clear();
    cut_started_ = false;
  };

  void Cut(std::vector<RobotInfo> *robot_info,
           ShelfManager *shelf_manager,
           std::vector<ActionWithTimeSeq> *remaining_plan,
           int &start_time_ms);
  void SetPlan(const std::vector<ActionWithTimeSeq> &new_plan,
               const std::set<Edge> &edges,
               const std::set<Edge> &ref_edges);
  // Returns the to acknowledge and to send part of the current plan.
  // The initial status of all robots corresponds to this plan is available in robot manager.
  ActionPlan GetCurrentPlan() const;
  void UpdateRobotStatus(int robot_id, Action a);
  std::vector<std::vector<Action>> GetCommands(const std::vector<RobotInfo> &robot_info, int &acked);

 private:
  const int robot_count_;

  // Initialized to 0, when this value is equal to the action count, all the actions are sent.
  std::vector<int> to_send_action_index_;
  // Initialized to -1, when this is equal to <last_action_index_>, this plan is considered done.
  std::vector<int> replied_action_index_;

  std::vector<std::vector<ActionWithTime>> plan_;
  TwoWayAdjList adj_;
  bool cut_started_;
};

// Helper functions.
std::set<Edge> GetNewDependency(const std::vector<ActionWithTimeSeq> &plan,
                                const std::vector<ActionWithTimeSeq> &remaining_plan);
std::set<Edge> GetDependencyWithinPlan(const std::set<int> &robots,
                                       const std::vector<ActionWithTimeSeq> &plan);
std::set<Edge> GetDependencyInterPlan(const std::set<int> &robots_0,
                                      const std::vector<ActionWithTimeSeq> &plan_0,
                                      const std::set<int> &robots_1,
                                      const std::vector<ActionWithTimeSeq> &plan_1);
std::set<Edge> BuildDependencyOld(const std::vector<ActionWithTimeSeq> &plan);
}

#endif