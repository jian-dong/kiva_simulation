#ifndef KIVA_SIMULATION_SRC_KS_SIMULATOR_H_
#define KIVA_SIMULATION_SRC_KS_SIMULATOR_H_

#include <vector>
#include <mutex>
#include <string>
#include <queue>

#include "common_types.h"
#include "utilities.h"
#include "interface/scheduler_simulator_types.h"
#include "hiredis/hiredis.h"
#include "constants.h"
#include "ks_map.h"
#include "ks_scheduler_common.h"

namespace ks {

class KsScheduler;

// Location with x, y as double values.
struct LocationDouble {
  double x, y;
  LocationDouble() { x = -1; y = -1; };
  LocationDouble(double x, double y) : x(x), y(y) {};

  LocationDouble &operator=(const LocationDouble &o) {
    x = o.x;
    y = o.y;
  }

  Location GetLocation() {
    return {(int)round(x), (int)round(y)};
  }
};

inline LocationDouble operator+(const LocationDouble& p, const std::pair<int, int>& o) {
  return LocationDouble(p.x + o.first, p.y + o.second);
}

inline LocationDouble operator-(const LocationDouble& p, const std::pair<int, int>& o) {
  return LocationDouble(p.x - o.first, p.y - o.second);
}


struct ActionProgress {
  // TODO: change single action to action group in the future, to simulate accelation etc.
  Action action;
  TimePoint start_time;
  TimePoint end_time;

  ActionProgress(Action a) : action(a) {
    start_time = kEpoch;
    end_time = kEpoch;
  };

  bool Started() {
    return start_time != kEpoch;
  }

  bool Finished() {
    return Started() && (end_time < GetCurrentTime());
  }

  // Return progress, a double in [0, 1].
  double GetProgress(TimePoint cur) const {
//    return (double)(cur - start_time) / (double)(end_time - start_time);
    return 0;
  }
};

// Represents the robot status in the physical world.
struct RobotStatus {
  int id;
  Direction dir;
  LocationDouble loc;
  bool shelf_attached;
  int shelf_id;

  std::queue<ActionProgress> pending_actions;

  RobotStatus() = default;
  RobotStatus(const RobotInfo& r) {
    id = r.id;
    dir = r.pos.dir;
    loc.x = r.pos.loc.x;
    loc.y = r.pos.loc.y;
    shelf_attached = false;
  }

  void OutputStatus(std::stringstream& str, const TimePoint& cur_time) {
    double x = loc.x, y = loc.y;
    double dir_rad = kDirectionToRadian.at(dir);
    double x_modifier = 0, y_modifier = 0, dir_modifier = 0;
    if (!pending_actions.empty()) {
      const ActionProgress& p = pending_actions.front();
      double progress = p.GetProgress(cur_time);
      if (p.action == Action::MOVE) {
        const auto& delta = kDirectionToDelta.at(dir);
        x_modifier = delta.first * progress;
        y_modifier = delta.second * progress;
      }
      if (p.action == Action::CTURN) {
        dir_modifier = -kPi / 2 * progress;
      }
      if (p.action == Action::CCTURN) {
        dir_modifier = kPi / 2 * progress;
      }
    }
    x += x_modifier;
    y += y_modifier;
    dir_rad += dir_modifier;

//    str << std::to_string(x) << " " << std::to_string(y) << " " << std::to_string(dir_rad)
//        << " " << std::to_string(shelf_attached) << " ";
  }
};

class KsSimulator {
 public:
  KsSimulator() = default;
  void Init(KsScheduler *scheduler_p, const KsMap &ks_map);
  void Run();
  void AddActions(Command action_seq);

 private:
  void UpdateWithAction(RobotStatus &r, Action a);
  void RedisSet(const std::string &key, const std::string &value);

  // IO related fields. Once a message queue
  std::mutex mutex_io_;
  std::queue<Command> mq_from_scheduler_;

  KsScheduler* scheduler_p_;

  // The simulator has only one thread, which periodically check the newly added action,
  // update robot status and write to redis. so there is no need to lock the main data
  // structure. We only need to lock the io related fields.
  std::vector<RobotStatus> robot_status_;

  // Index of this array is the shelf id, value of this array is the shelf location. (-1, -1) for
  // shelf on a robot.
  std::vector<Location> shelf_id_to_loc_;
  std::map<Location, int> loc_to_shelf_id_;

  redisContext *redis_;
};

}

#endif