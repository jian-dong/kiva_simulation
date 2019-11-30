#include "ks_simulator.h"

#include <sstream>
#include <iomanip>

#include "utilities.h"
#include "hiredis/hiredis.h"
#include "logger.h"

namespace ks {

using namespace std;
using std::chrono::milliseconds;

namespace {
milliseconds GetActionDuration(Action a) {
  if (a == Action::CCTURN || a == Action::CTURN) {
    return milliseconds(kTurnDurationMs); // + milliseconds(GenRandomNumber(-500, 600));
  }
  if (a == Action::DETACH || a == Action::ATTACH) {
    return milliseconds(kAttachDetachDurationMs);
  }
  if (a == Action::MOVE) {
    return milliseconds(kMoveDurationMs); //+ milliseconds((int)pow(GenRandomNumber(0, 50), 2) );
  }
  if (a == Action::YIELD) {
    return milliseconds(kWaitDurationMs);
  }
}
}

void KsSimulator::Init(KsSchedulerApi *scheduler_p, const KsMap &ks_map) {
  robot_count_ = ks_map.robot_count_;
  scheduler_p_ = scheduler_p;

  const vector<Location> &shelf_storage_points = ks_map.GetShelfStoragePoints();
  for (int i = 0; i < robot_count_; i++) {
    robot_status_.emplace_back(i, shelf_storage_points[i]);
  }

  shelf_id_to_loc_.resize(ks_map.shelf_count_);
  loc_to_shelf_id_.clear();
  for (int i = 0; i < ks_map.shelf_count_; i++) {
    shelf_id_to_loc_[i] = ks_map.GetShelfStoragePoints()[i];
    loc_to_shelf_id_[ks_map.GetShelfStoragePoints()[i]] = i;
  }

  redis_ = redisConnect(kRedisHostname, kRedisPort);
  if (redis_ == nullptr || redis_->err) {
    LogFatal("Failed to connect to redis.");
  }

  robot_status_sanity_check_.resize(robot_count_);
}

void KsSimulator::AddActions(Command command) {
  lock_guard<mutex> lock(mutex_io_);
  mq_from_scheduler_.push(command);
}

void KsSimulator::Run() {
  while (true) {
    mutex_io_.lock();

    while (!mq_from_scheduler_.empty()) {
      const Command& command = mq_from_scheduler_.front();
      for (const auto& action : command.actions) {
        TimePoint prev_action_finish_time = kEpoch;
        if (!robot_status_[command.robot_id].pending_actions.empty()) {
          prev_action_finish_time = robot_status_[command.robot_id].pending_actions.back().end_time;
        }
        ActionProgress progress(action);
        progress.start_time = prev_action_finish_time == kEpoch ? GetCurrentTime() : prev_action_finish_time;
        progress.end_time = progress.start_time + GetActionDuration(action);
        robot_status_[command.robot_id].pending_actions.push(progress);
      }
      mq_from_scheduler_.pop();
    }
    mutex_io_.unlock();

    // Process each robot, report if an action is finished.
    for (RobotStatus& r : robot_status_) {
      while (!r.pending_actions.empty()) {
        if (r.pending_actions.front().Finished()) {
          scheduler_p_->ReportActionDone({r.id, r.pending_actions.front().action});
          UpdateRobotStatusWithAction(r.pending_actions.front().action, &r);
          r.pending_actions.pop();
        } else {
          break;
        }
      }
    }

    const auto cur_time = GetCurrentTime();
    std::stringstream sstream;
    sstream << std::setprecision(2) << std::fixed;
    for (RobotStatus& r : robot_status_) {
      const auto& s = r.OutputStatus(cur_time);
      robot_status_sanity_check_[s.id] = s;
      sstream << s.loc.x << " " << s.loc.y << " " << s.direction
          << " " << s.shelf_attached << " ";
    }
    for (const auto& loc : shelf_id_to_loc_) {
      sstream << loc.x << " " << loc.y << " ";
    }

    auto redis_start = std::chrono::system_clock::now();
    RedisSet(kRedisKey, sstream.str());
    auto redis_end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = redis_end - redis_start;
//    std::cout << "Redis set elapsed time: " << elapsed_seconds.count() << "s\n";

    SanityCheck();
    SleepMS(kSimulatorSleepDurationMs);
  }
}

void KsSimulator::UpdateRobotStatusWithAction(Action a, RobotStatus *r) {
  switch (a) {
    case Action::ATTACH:
      assert(shelf_id_to_loc_[loc_to_shelf_id_.at(r->loc.GetLocation())] == r->loc.GetLocation());
      r->shelf_attached = true;
      r->shelf_id = loc_to_shelf_id_.at(r->loc.GetLocation());

      shelf_id_to_loc_[r->shelf_id] = kInvalidLocation;
      loc_to_shelf_id_.erase(r->loc.GetLocation());
      return;
    case Action::DETACH:
      r->shelf_attached = false;
      shelf_id_to_loc_[r->shelf_id] = r->loc.GetLocation();
      loc_to_shelf_id_[r->loc.GetLocation()] = r->shelf_id;
      return;
    case Action::YIELD:
      return;
    case Action::MOVE:
      r->loc = r->loc + kDirectionToDelta.at(r->dir);
      return;
    case Action::CTURN:
      r->dir = ClockwiseTurn(r->dir);
      return;
    case Action::CCTURN:
      r->dir = CounterClockwiseTurn(r->dir);
      return;
    default:
      LogFatal("Invalid action.");
  }
}

void KsSimulator::RedisSet(const string &key, const string &value) {
  redisReply *reply;
  reply = (redisReply *) redisCommand(redis_, "SET %s %s", key.c_str(), value.c_str());
  freeReplyObject(reply);
}

void KsSimulator::SanityCheck() {
  for (int i = 0; i < robot_count_; i++) {
    for (int j = i + 1; j < robot_count_; j++) {
      if (GetDist(robot_status_sanity_check_[i].loc, robot_status_sanity_check_[j].loc) < 0.8) {
        LogFatal("Collision.");
      }
    }
  }
}

}