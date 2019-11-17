#include "ks_scheduler.h"

#include <set>
#include <map>

#include "ks_simulator.h"
#include "ks_wms.h"
#include "path_finder/sipp_solver.h"
#include "utilities.h"

namespace ks {

using namespace std;

namespace {
void ApplyActions(const map<int, vector<Action>> &robot_to_actions, vector<RobotInfo> *robot_info) {
  for (const auto &[robot_id, actions] : robot_to_actions) {
    for (Action a : actions) {
      ApplyActionOnRobot(a, &(*robot_info)[robot_id]);
    }
  }
}

}

void KsScheduler::AddMission(WmsMission mission) {
  lock_guard<mutex> lock(mutex_io_up_);
  missions_to_assign_.insert(mission);
}

void KsScheduler::ReportActionDone(CommandReport r) {
  lock_guard<mutex> lock(mutex_io_down_);
  robot_report_.push(r);
}

void KsScheduler::Init(KsWms *wms_p, KsSimulator *simulator_p) {
  wms_p_ = wms_p;
  simulator_p_ = simulator_p;

  robot_manager_.Init(wms_p);

  sipp_p_ = new SippSolver(map_, &shelf_manager_);
}

void KsScheduler::Run() {
  // The main event loop for scheduler.
  // 1. Assign mission to idle robots.
  // 2. If there are any new assignment, call GetCut(), to get the status of all current robots, and combine all
  // robots with assignment, for the new round of path planning.
  // 3. Give the whole new plan to executor.
  mutex_.lock();
  while (true) {
    mutex_.unlock();
    SleepMS(kScheduleIntervalMs);

    mutex_io_up_.lock();
    if (missions_to_assign_.empty()) {
      mutex_io_up_.unlock();
      continue;
    }

    mutex_.lock();
    robot_manager_.AssignMissions(missions_to_assign_);
    mutex_io_up_.unlock();

    map<int, vector<Action>> actionsTillCut = action_graph_.GetCutInternal();
    vector<RobotInfo> robot_info = robot_manager_.GetRobotInfo();

    mutex_.unlock();
    ApplyActions(actionsTillCut, &robot_info);

    PfResponse resp = sipp_p_->FindPath({robot_info});
    mutex_.lock();
    action_graph_.SetPlan(resp.plan);
  }
}

void KsScheduler::AdgRunner() {
  mutex_.lock();
  while (true) {
    mutex_.unlock();

    // Send all pending messages to wms.
    for (const MissionReport &tmp : mq_to_wms_) {
      wms_p_->ReportMissionStatus(tmp);
    }
    mq_to_wms_.clear();

    SleepMS(kUpdateIntervalMs);

    mutex_io_down_.lock();
    std::queue<CommandReport> tmp_robot_report = robot_report_;
    while (!robot_report_.empty()) {
      robot_report_.pop();
    }
    mutex_io_down_.unlock();

    mutex_.lock();
    while (!tmp_robot_report.empty()) {
      CommandReport report = tmp_robot_report.front();
      tmp_robot_report.pop();
      // Robot manager and action graph need to be updated in sync.
      const auto &rtn = robot_manager_.UpdateRobotStatus(report.robot_id, report.action);
      if (rtn.has_value()) {
        mq_to_wms_.push_back(rtn.value());
      }
      const auto &actions = action_graph_.UpdateRobotStatus(report.robot_id, report.action);
      if (!actions.empty()) {
        simulator_p_->AddActions({report.robot_id, actions});
      }
    }
  }
}

}