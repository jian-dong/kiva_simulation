#include "ks_wms.h"

#include <string>
#include <vector>
#include <cassert>

#include "constants.h"
#include "logger.h"
#include "utilities.h"

namespace ks {

using namespace std;

namespace {
// Select a random qualified element with reservior sampling.
template<class T>
std::optional<T> ReserviorSampling(const vector<T> &array, bool (T::*predicator)(void) const) {
  T selected;
  bool found = false;
  int counter = 1;
  for (const auto &elem : array) {
    if ((elem.*predicator)()) {
      if (GetTrueWithProb(1, counter)) {
        selected = elem;
        found = true;
      }
      counter++;
    }
  }
  if (found) {
    return selected;
  } else {
    return nullopt;
  }
}
}

void KsWms::Init(KsSchedulerApi *scheduler_p) {
  scheduler_p_ = scheduler_p;
  for (int i = 0; i < ks_map_.operation_point_count_; i++) {
    operation_point_info_.emplace_back(i);
  }
  for (int i = 0; i < ks_map_.storage_point_count_; i++) {
    storage_point_info_.emplace_back(i);
  }

  shelf_info_.resize(ks_map_.shelf_count_);
  mission_id_counter_ = 0;
  move_to_op_mission_count_ = 0;

  // The way shelves are initialized.
  for (int i = 0; i < ks_map_.shelf_count_; i++) {
    storage_point_info_[i].shelf_id = i;
  }
}

void KsWms::Run() {
  while (true) {
    mutex_.lock();
    ProcessReports();
    GenOpToSpMissions();
    GenSpToOpMissions();
    mutex_.unlock();
    SleepMS(kMissionGenerationIntervalMs);
  }
}

void KsWms::GenSpToOpMissions() {
  while (move_to_op_mission_count_ < kPendingMoveToOperationMissionLimit) {
    // Generate a mission.
    WmsMission m(mission_id_counter_++);

    // Select from and to.
    auto from_or_empty = ReserviorSampling(storage_point_info_, &StoragePointInfo::CapableForMoveOut);
    auto to_or_empty = ReserviorSampling(operation_point_info_, &OperationPointInfo::CapableForMoveIn);
    if (!from_or_empty.has_value() || !to_or_empty.has_value()) {
      break;
    }
    StoragePointInfo from = from_or_empty.value();
    OperationPointInfo to = to_or_empty.value();

    // Reserve the from shelf and to location.
    storage_point_info_[from.index].scheduled_to_change = true;
    operation_point_info_[to.index].scheduled_to_change = true;

    m.shelf_id = from.shelf_id;
    assert(from.shelf_id != -1);
    m.pick_from.type = LocationType::STORAGE_POINT;
    m.pick_from.index = from.index;
    m.pick_from.loc = shelf_storage_points_[from.index];

    m.drop_to.type = LocationType::OPERATION_POINT;
    m.drop_to.index = to.index;
    m.drop_to.loc = shelf_operation_points_[to.index];

    pending_missions_.insert(m);
    scheduler_p_->AddMission(m);
    move_to_op_mission_count_++;
  }
}

void KsWms::GenOpToSpMissions() {
  for (const OperationPointInfo &op_info : operation_point_info_) {
    if (op_info.CapableForMoveOut()
        && ElapsedTimeLongerThanMs(op_info.start_time, kMinOperationTimeS)) {
      // If the shelf had stayed at the operation point for some time, move it out with some probability.
      if (GetTrueWithProb(kMoveOutProb)) {
        WmsMission m(mission_id_counter_++);
        const OperationPointInfo &from = op_info;
        auto to_or_empty = ReserviorSampling(storage_point_info_, &StoragePointInfo::CapableForMoveIn);
        if (!to_or_empty.has_value()) {
          // TODO: this should not happen.
          break;
        }
        const StoragePointInfo to = to_or_empty.value();
        operation_point_info_[from.index].scheduled_to_change = true;
        storage_point_info_[to.index].scheduled_to_change = true;

        m.shelf_id = from.shelf_id;
        assert(from.shelf_id != -1);
        m.pick_from.type = LocationType::OPERATION_POINT;
        m.pick_from.index = from.index;
        m.pick_from.loc = shelf_operation_points_[from.index];

        m.drop_to.type = LocationType::STORAGE_POINT;
        m.drop_to.index = to.index;
        m.drop_to.loc = shelf_storage_points_[to.index];

        pending_missions_.insert(m);
        scheduler_p_->AddMission(m);
      }
    }
  }
}

// Internal helper, this function needs to be called with the mutex held.
const WmsMission &KsWms::GetPendingMission(int id) {
  WmsMission key(id);
  if (pending_missions_.find(key) == pending_missions_.end()) {
    LogFatal("Failed to find mission with id: " + to_string(id));
  }
  return *(pending_missions_.find(key));
}

void KsWms::ReportMissionStatus(MissionReport r) {
  lock_guard<mutex> lock(mutex_io_);
//  cout << "Mission :" << r.mission.id << " done." << endl;
  message_queue_.push(r);
}

void KsWms::ProcessReports() {
  mutex_io_.lock();
  // Copy incoming messages.
  queue<MissionReport> tmp_mq = message_queue_;
  while (!message_queue_.empty()) {
    message_queue_.pop();
  }
  mutex_io_.unlock();

  while (!tmp_mq.empty()) {
    MissionReport report = tmp_mq.front();
    tmp_mq.pop();
    const WmsMission &m = GetPendingMission(report.mission.id);
    if (report.type == MissionReportType::PICKUP_DONE) {
      if (m.pick_from.type == LocationType::STORAGE_POINT) {
        assert(storage_point_info_[m.pick_from.index].shelf_id != -1);
        storage_point_info_[m.pick_from.index].shelf_id = -1;
        storage_point_info_[m.pick_from.index].scheduled_to_change = false;
      } else if (m.pick_from.type == LocationType::OPERATION_POINT) {
        assert(operation_point_info_[m.pick_from.index].shelf_id != -1);
        operation_point_info_[m.pick_from.index].shelf_id = -1;
        operation_point_info_[m.pick_from.index].scheduled_to_change = false;
      } else {
        exit(0);
      }
    } else if (report.type == MissionReportType::MISSION_DONE) {
      if (m.drop_to.type == LocationType::STORAGE_POINT) {
        assert(storage_point_info_[m.drop_to.index].shelf_id == -1);
        storage_point_info_[m.drop_to.index].shelf_id = m.shelf_id;
        storage_point_info_[m.drop_to.index].scheduled_to_change = false;
      } else if (m.drop_to.type == LocationType::OPERATION_POINT) {
        assert(operation_point_info_[m.drop_to.index].shelf_id == -1);
        operation_point_info_[m.drop_to.index].shelf_id = m.shelf_id;
        operation_point_info_[m.drop_to.index].start_time = GetCurrentTime();
        operation_point_info_[m.drop_to.index].scheduled_to_change = false;
        move_to_op_mission_count_--;
      } else {
        exit(0);
      }
      assert(pending_missions_.erase({report.mission.id}) == 1);
    } else {
      exit(0);
    }
  }
}
}