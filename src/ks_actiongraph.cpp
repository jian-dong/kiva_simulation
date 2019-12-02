#include "ks_actiongraph.h"

#include <algorithm>

#include "logger.h"

namespace ks {

using namespace std;

void KsActionGraph::Cut(vector<RobotInfo> *robot_info,
                        ShelfManager *shelf_manager,
                        vector<ActionWithTimeSeq> *remaining_plan) {
  cur_plan_p_->Cut(robot_info, shelf_manager, remaining_plan);
}

void KsActionGraph::SetPlan(const vector<RobotInfo> &prev_robot_info,
    const vector<ActionWithTimeSeq> &plan) {
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
//    cout << "swap plan 1" << endl;
    swap(cur_plan_p_, next_plan_p_);
  } else {
    if (cur_plan_p_->Finished(robot_info)) {
      cur_plan_p_->Clear();
//      cout << "current plan finished, swap plan 2" << endl;
      swap(cur_plan_p_, next_plan_p_);
    }
  }

  vector<vector<Action>> rtn(robot_count_);
  for (int i = 0; i < robot_count_; i++) {
    rtn[i] = cur_plan_p_->GetActionToSend(i);
  }
  return rtn;
}

ActionPlan KsActionGraph::GetCurrentPlan() {
  return cur_plan_p_->GetCurrentPlan();
}

void GlobalPlan::Cut(vector<RobotInfo> *robot_info,
                     ShelfManager *shelf_manager,
                     vector<ActionWithTimeSeq> *remaining_plan) {
  if (cut_) {
    *robot_info = cached_robot_info_;
    *shelf_manager = cached_shelf_manager_;
    *remaining_plan = cached_remaining_plan_;
    return;
  } else {
    cut_ = true;
  }

  for (int i = 0; i < robot_count_; i++) {
    for (int j = to_send_action_index_[i]; j < ((int)plan_[i].size()); j++) {
      (*remaining_plan)[i].push_back(plan_[i][j]);
    }
    plan_[i].resize(to_send_action_index_[i]);
  }

  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    for (int j = replied_action_index_[robot_id] + 1; j < (int)to_send_action_index_[robot_id]; j++) {
      assert(((int)plan_[robot_id].size()) == ((int)to_send_action_index_[robot_id]));
      Action a = plan_[robot_id][j].action;
      ApplyActionOnRobot(a, &((*robot_info)[robot_id]), shelf_manager);
    }
  }

  target_robot_info_ = *robot_info;

  cached_robot_info_ = *robot_info;
  cached_shelf_manager_ = *shelf_manager;
  cached_remaining_plan_ = *remaining_plan;
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
      ApplyActionOnRobot(awt.action, &init_robot_info[i], nullptr);
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
  assert(replied_action_index_[robot_id] < to_send_action_index_[robot_id]);
  adj_.RemoveAllEdgesFrom({robot_id, replied_action_index_[robot_id]});
}

ActionPlan GlobalPlan::GetCurrentPlan() {
  ActionPlan rtn(robot_count_);

  // TODO: consider whether unreplied actions or to send action should be used.
  for (int i = 0; i < robot_count_; i++) {
    for (int j = replied_action_index_[i] + 1; j < ((int)plan_[i].size()); j++) {
      rtn[i].push_back(plan_[i][j]);
    }
  }

//  for (int i = 0; i < robot_count_; i++) {
//    for (int j = to_send_action_index_[i]; j < ((int)plan_[i].size()); j++) {
//      rtn[i].push_back(plan_[i][j]);
//    }
//  }
  return rtn;



}

}
