#include "ks_actiongraph.h"

#include <algorithm>

#include "logger.h"

namespace ks {

using namespace std;

void KsActionGraph::Cut(vector<RobotInfo> &robot_info,
                        ShelfManager &shelf_manager,
                        vector<ActionWithTimeSeq> &remaining_plan) {
  assert(robot_info.size() == robot_count_);
  cur_plan_p_->Cut(robot_info, shelf_manager, remaining_plan);
}

void KsActionGraph::SetPlan(const vector<RobotInfo> &prev_robot_info,
    const vector<ActionWithTimeSeq> &plan) {
  assert(plan.size() == robot_count_);
  if (!cur_plan_p_->target_robot_info_.empty()) {
    if (cur_plan_p_->target_robot_info_ != prev_robot_info) {
      LogFatal("Invalid plan.");
      return;
    }
  }
  next_plan_p_->SetPlan(prev_robot_info, plan);
}

void KsActionGraph::UpdateRobotStatus(int robot_id, Action a) {
  cur_plan_p_->UpdateRobotStatus(robot_id, a);
}

vector<vector<Action>> KsActionGraph::GetCommands(const std::vector<RobotInfo> &robot_info) {
  if (!cur_plan_p_->plan_set_) {
    // For the case that both current and next plan are not set.
    swap(cur_plan_p_, next_plan_p_);
  } else {
    if (cur_plan_p_->Finished(robot_info)) {
      cur_plan_p_->Clear();
      swap(cur_plan_p_, next_plan_p_);
    }
  }

  vector<vector<Action>> rtn(robot_count_);
  for (int i = 0; i < robot_count_; i++) {
    rtn[i] = cur_plan_p_->GetActionToSend(i);
  }
  return rtn;
}

void GlobalPlan::Cut(vector<RobotInfo> &robot_info,
                     ShelfManager &shelf_manager,
                     vector<ActionWithTimeSeq> &remaining_plan) {
  for (int i = 0; i < robot_count_; i++) {
    for (int j = to_send_action_index_[i]; j < (int)plan_[i].size(); j++) {
      remaining_plan[i].push_back(plan_[i][j]);
    }
    plan_[i].resize(to_send_action_index_[i]);
  }

  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    for (int j = replied_action_index_[robot_id] + 1; j < to_send_action_index_[robot_id]; j++) {
      Action a = plan_[robot_id][j].action;
      ApplyActionOnRobot(a, &(robot_info[robot_id]));
      if (a == Action::ATTACH) {
        shelf_manager.RemoveMapping(robot_info[robot_id].mission.wms_mission.shelf_id,
                                    robot_info[robot_id].mission.wms_mission.pick_from.loc);
      } else if (a == Action::DETACH) {
        shelf_manager.AddMapping(robot_info[robot_id].mission.wms_mission.shelf_id,
                                 robot_info[robot_id].mission.wms_mission.drop_to.loc);
      }
    }
  }

  target_robot_info_ = robot_info;
}

void GlobalPlan::SetPlan(vector<RobotInfo> init_robot_info, const vector<ActionWithTimeSeq> &plan) {
  Clear();
  plan_ = plan;

  // Add type II edges, type I edges(between actions of the same robot) are implicit.
  for (int robot_0 = 0; robot_0 < robot_count_; robot_0++) {
    for (int action_index_0 = 0; action_index_0 < (int)plan[robot_0].size(); action_index_0++) {
      for (int robot_1 = 0; robot_1 < robot_count_; robot_1++) {
        if (robot_0 == robot_1) {
          continue;
        }
        for (int action_index_1 = 0; action_index_1 < (int)plan[robot_1].size(); action_index_1++) {
          ActionWithTime awt_0 = plan[robot_0][action_index_0];
          ActionWithTime awt_1 = plan[robot_1][action_index_1];
          if (awt_0.start_pos.loc == awt_1.end_pos.loc && awt_0.start_time_ms <= awt_1.start_time_ms) {
            assert(awt_0.start_time_ms < awt_1.end_time_ms);
//            cout << "Adding edge, from: " << from.to_string() << " to: " << to.to_string() << endl;
            adj_.AddEdge({robot_0, action_index_0}, {robot_1, action_index_1});
          }
        }
      }
    }
  }

  for (int i = 0; i < robot_count_; i++) {
    for (ActionWithTime awt : plan_[i]) {
      ApplyActionOnRobot(awt.action, &init_robot_info[i]);
    }
  }
  target_robot_info_ = init_robot_info;
  plan_set_ = true;
}

vector<Action> GlobalPlan::GetActionToSend(int robot_id) {
  vector<Action> rtn;
  int action_index;
  for (action_index = to_send_action_index_[robot_id]; action_index < (int) plan_[robot_id].size(); action_index++) {
    if (action_index - replied_action_index_[robot_id] > 5) {
      break;
    }
    Node to_send(robot_id, action_index);
    if (adj_.CanSendAction(to_send)) {
      rtn.push_back(plan_[robot_id][action_index].action);
    } else {
      break;
    }
  }
  to_send_action_index_[robot_id] = action_index;
  return rtn;
}

void GlobalPlan::UpdateRobotStatus(int robot_id, Action a) {
  assert(plan_[robot_id][replied_action_index_[robot_id] + 1].action == a);
  replied_action_index_[robot_id]++;
  adj_.RemoveAllEdgesFrom({robot_id, replied_action_index_[robot_id]});
}

}
