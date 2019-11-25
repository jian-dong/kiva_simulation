#include "ks_actiongraph.h"

#include <algorithm>

#include "logger.h"

namespace ks {

using namespace std;

void KsActionGraph::Cut(vector<RobotInfo> &robot_info, ShelfManager &shelf_manager) {
  assert(robot_info.size() == robot_count_);
  cur_plan_p_->Cut(robot_info, shelf_manager);
}

void KsActionGraph::SetPlan(const vector<ActionWithTimeSeq> &plan) {
  assert(plan.size() == robot_count_);
  next_plan_p_->SetPlan(plan);
}

void KsActionGraph::UpdateRobotStatus(int robot_id, Action a) {
  cur_plan_p_->UpdateRobotStatus(robot_id, a);
}

vector<vector<Action>> KsActionGraph::GetCommands() {
  if (cur_plan_p_->Finished()) {
    swap(cur_plan_p_, next_plan_p_);
  }

  vector<vector<Action>> rtn(robot_count_);
  if (cur_plan_p_->Finished()) {
    return rtn;
  }

  for (int i = 0; i < robot_count_; i++) {
    rtn[i] = cur_plan_p_->GetActionToSend(i);
  }
  return rtn;
}

void GlobalPlan::Cut(vector<RobotInfo> &robot_info, ShelfManager &shelf_manager) {
  if (scheduled_to_change_) {
    robot_info = cached_robot_info_;
    return;
  } else {
    scheduled_to_change_ = true;
  }

  for (int i = 0; i < robot_count_; i++) {
    plan_[i].resize(to_send_action_index_[i]);
  }

  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    for (int j = replied_action_index_[robot_id] + 1; j < to_send_action_index_[robot_id]; j++) {
      Action a = plan_[robot_id][j];
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

  cached_robot_info_ = robot_info;
}

void GlobalPlan::SetPlan(const vector<ActionWithTimeSeq> &plan) {
  Clear();
  for (int i = 0; i < robot_count_; i++) {
    for (ActionWithTime a : plan[i]) {
      plan_[i].push_back(a.action);
    }
  }

  // Add type II edges, type I edges(between actions of the same robot) are implicit.
  for (int robot_0 = 0; robot_0 < robot_count_; robot_0++) {
    for (int action_index_0 = 0; action_index_0 < plan[robot_0].size(); action_index_0++) {
      for (int robot_1 = 0; robot_1 < robot_count_; robot_1++) {
        if (robot_0 == robot_1) {
          continue;
        }
        for (int action_index_1 = 0; action_index_1 < plan[robot_1].size(); action_index_1++) {
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
}

vector<Action> GlobalPlan::GetActionToSend(int robot_id) {
  vector<Action> rtn;
  int action_index;
  for (action_index = to_send_action_index_[robot_id]; action_index < (int) plan_[robot_id].size(); action_index++) {
    Node to_send(robot_id, action_index);
    if (adj_.CanSendAction(to_send)) {
      rtn.push_back(plan_[robot_id][action_index]);
    } else {
      break;
    }
  }
//  cout << "rtn size: " << rtn.size() << " action index: " << action_index << endl;
  to_send_action_index_[robot_id] = action_index;
  return rtn;
}

void GlobalPlan::UpdateRobotStatus(int robot_id, Action a) {
  assert(plan_[robot_id][replied_action_index_[robot_id] + 1] == a);
  replied_action_index_[robot_id]++;
  adj_.RemoveAllEdgesFrom({robot_id, replied_action_index_[robot_id]});
}

}
