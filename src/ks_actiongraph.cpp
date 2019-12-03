#include "ks_actiongraph.h"

#include <algorithm>

#include "logger.h"

namespace ks {

using namespace std;

void KsActionGraph::Cut(vector<RobotInfo> *robot_info,
                        ShelfManager *shelf_manager,
                        vector<ActionWithTimeSeq> *remaining_plan) {
  assert(!in_transition_);
  in_transition_ = true;
  cout << "cut happened" << endl;
  cur_plan_p_->Cut(robot_info, shelf_manager, remaining_plan);
}

void KsActionGraph::SetPlan(const std::vector<RobotInfo> &prev_robot_info,
                            const std::vector<ActionWithTimeSeq> &plan,
                            TwoWayAdjList *adj_p) {
  if (cur_plan_p_->plan_set_) {
    if (cur_plan_p_->target_robot_info_ != prev_robot_info) {
      std::cout << "target robot info: " << std::endl;
      PrintRobotInfo(cur_plan_p_->target_robot_info_);
      std::cout << "actual robot info: " << std::endl;
      PrintRobotInfo(prev_robot_info);
      std::cout << std::endl;

      LogFatal("Invalid plan.");
      return;
    }
  }
  cout << "set plan happened" << endl;
  next_plan_p_->SetPlan(prev_robot_info, plan, adj_p);
  assert(in_transition_);
  in_transition_ = false;
}

void KsActionGraph::UpdateRobotStatus(int robot_id, Action a) {
  cur_plan_p_->UpdateRobotStatus(robot_id, a);
}

vector<vector<Action>> KsActionGraph::GetCommands(const std::vector<RobotInfo> &robot_info, int &acked) {
  vector<vector<Action>> rtn(robot_count_);
  if (!cur_plan_p_->plan_set_) {
    if (next_plan_p_->plan_set_) {
      cout << "swap plan happened - should not happen often" << endl;
//      swap(cur_plan_p_, next_plan_p_);
      GlobalPlan *tmp_p = cur_plan_p_;
      cur_plan_p_ = next_plan_p_;
      next_plan_p_ = tmp_p;
      acked = 0;
    } else {
      return rtn;
    }
  } else {
    if (cur_plan_p_->Finished(robot_info) && (!in_transition_)) {
      if (next_plan_p_->plan_set_) {
        cur_plan_p_->plan_set_ = false;
        cout << "swap plan happened" << endl;
//        swap(cur_plan_p_, next_plan_p_);
        GlobalPlan *tmp_p = cur_plan_p_;
        cur_plan_p_ = next_plan_p_;
        next_plan_p_ = tmp_p;
        cout << "total acked in current plan: " << acked << endl;
        acked = 0;
      } else {
        return rtn;
      }
    }
  }

  for (int i = 0; i < robot_count_; i++) {
    rtn[i] = cur_plan_p_->GetActionToSend(i);
  }
  return rtn;
}

ActionPlan KsActionGraph::GetCurrentPlan() const {
  return cur_plan_p_->GetCurrentPlan();
}

TwoWayAdjList KsActionGraph::BuildTwoWayAdjList(const std::vector<ActionWithTimeSeq> &plan) {
  int robot_count = plan.size();
  TwoWayAdjList rtn;
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
            rtn.AddEdge({robot_0, action_index_0}, {robot_1, action_index_1});
            break;
          }
        }
      }
    }
  }
  return rtn;
}

void GlobalPlan::Cut(vector<RobotInfo> *robot_info,
                     ShelfManager *shelf_manager,
                     vector<ActionWithTimeSeq> *remaining_plan) {
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
}

void GlobalPlan::SetPlan(vector<RobotInfo> init_robot_info,
                         const vector<ActionWithTimeSeq> &plan,
                         TwoWayAdjList *adj_p) {
  assert(!plan_set_);
  Clear();
  plan_ = plan;
  cout << "newly set plan size: " << Get2DMatrixSize(plan_) << endl;
  adj_ = std::move(*adj_p);

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
    if (action_index - replied_action_index_[robot_id] > kCommandsToSent) {
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

ActionPlan GlobalPlan::GetCurrentPlan() const {
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
