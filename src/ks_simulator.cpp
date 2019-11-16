#include "ks_simulator.h"

#include <sstream>
#include <iomanip>

#include "utilities.h"
#include "hiredis/hiredis.h"
#include "logger.h"
#include "ks_scheduler.h"

namespace ks {

using namespace std;
using std::chrono::milliseconds;

namespace {
milliseconds GetActionDuration(Action a) {
  if (a == Action::CCTURN || a == Action::CTURN) {
    return milliseconds(kTurnDurationMs) + milliseconds(GenRandomNumber(-500, 600));
  }
  if (a == Action::DETACH || a == Action::ATTACH) {
    return milliseconds(kAttachDetachDurationMs);
  }
  if (a == Action::MOVE) {
    return milliseconds(kMoveDurationMs) + milliseconds((int)pow(GenRandomNumber(0, 90), 2) );
  }
  if (a == Action::YIELD) {
    return milliseconds(kWaitDurationMs);
  }
}

}

void KsSimulator::Init(KsScheduler *scheduler_p, const KsMap &ks_map) {
  scheduler_p_ = scheduler_p;
  const std::vector<RobotInfo>& robot_info = scheduler_p_->GetRobotInfo();

  for (int i = 0; i < robot_info.size(); i++) {
    robot_status_.emplace_back(robot_info[i]);
  }

  shelf_id_to_loc_.resize(kShelfCount);
  loc_to_shelf_id_.clear();
  for (int i = 0; i < kShelfCount; i++) {
    shelf_id_to_loc_[i] = ks_map.GetShelfStoragePoints()[i];
    loc_to_shelf_id_[ks_map.GetShelfStoragePoints()[i]] = i;
  }

  redis_ = redisConnect(kRedisHostname, kRedisPort);
  if (redis_ == nullptr || redis_->err) {
    LogFatal("Failed to connect to redis");
  }

}

void KsSimulator::AddActions(Command action_seq) {
  lock_guard<mutex> lock(mutex_io_);
  mq_from_scheduler_.push(action_seq);
}

void KsSimulator::Run() {
  mutex_io_.lock();
  std::queue<Command> tmp_mq(mq_from_scheduler_);
  while (!mq_from_scheduler_.empty()) {
    mq_from_scheduler_.pop();
  }
  mutex_io_.unlock();

  while (!tmp_mq.empty()) {
    const Command& action_seq = tmp_mq.front();
    tmp_mq.pop();
    for (const auto& a : action_seq.actions) {
      TimePoint prev_action_finish_time = kEpoch;
      if (!robot_status_[action_seq.robot_id].pending_actions.empty()) {
        prev_action_finish_time = robot_status_[action_seq.robot_id].pending_actions.back().end_time;
      }
      ActionProgress p(a);
      p.start_time = prev_action_finish_time == kEpoch ? GetCurrentTime() : prev_action_finish_time;
      p.end_time = p.start_time + GetActionDuration(a);
      robot_status_[action_seq.robot_id].pending_actions.push(p);
    }
  }

  // Process each robot, report actions done.
  for (RobotStatus& r : robot_status_) {
    while (!r.pending_actions.empty()) {
      if (r.pending_actions.front().Finished()) {
        scheduler_p_->ReportActionDone({r.id, r.pending_actions.front().action});
        UpdateWithAction(r, r.pending_actions.front().action);
        r.pending_actions.pop();
      }
    }
  }

  const auto cur_time = GetCurrentTime();
  std::stringstream sstream;
  sstream << std::fixed << std::setprecision(2);
  for (RobotStatus& r : robot_status_) {
    r.OutputStatus(sstream, cur_time);
  }
  for (const auto& loc : shelf_id_to_loc_) {
    sstream << loc.x << " " << loc.y << " ";
  }
  RedisSet(kRedisKey, sstream.str());
}

void KsSimulator::UpdateWithAction(RobotStatus &r, Action a) {
  switch (a) {
    case Action::ATTACH:
      assert(shelf_id_to_loc_[loc_to_shelf_id_.at(r.loc.GetLocation())] == r.loc.GetLocation());

      r.shelf_attached = true;
      r.shelf_id = loc_to_shelf_id_.at(r.loc.GetLocation());

      shelf_id_to_loc_[r.shelf_id] = kInvalidLocation;
      loc_to_shelf_id_.erase(r.loc.GetLocation());
    case Action::DETACH:
      r.shelf_attached = false;
      shelf_id_to_loc_[r.shelf_id] = r.loc.GetLocation();
      loc_to_shelf_id_[r.loc.GetLocation()] = r.shelf_id;
    case Action::YIELD:
      break;
    case Action::MOVE:
      r.loc = r.loc + kDirectionToDelta.at(r.dir);
      break;
    case Action::CTURN:
      r.dir = ClockwiseTurn(r.dir);
      break;
    case Action::CCTURN:
      r.dir = CounterClockwiseTurn(r.dir);
      break;
    default:
      exit(0);
  }
}

void KsSimulator::RedisSet(const string &key, const string &value) {
  redisReply *reply;
  reply = (redisReply *) redisCommand(redis_, "SET %s %s", key.c_str(), value.c_str());
  freeReplyObject(reply);
}

}