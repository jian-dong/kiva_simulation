#ifndef KIVA_SIMULATION_SRC_KS_WMS_H_
#define KIVA_SIMULATION_SRC_KS_WMS_H_

#include <set>
#include <mutex>
#include <queue>

#include "common_types.h"
#include "ks_map.h"
#include "interface/ks_api.h"

namespace ks {

// WMS specific types.
// A place holder for shelf info.
struct ShelfInfo {
  std::string external_id;
};

struct OperationPointInfo {
  // The shelf in this operation point. -1 for empty.
  int shelf_id;
  bool scheduled_to_change;
  // The time point the shelf was moved into the operation point, in the format of seconds since epoch.
  TimePoint start_time;

  // Index for this operation point. For both the OperationPointInfo array and
  // the operation point location array. (This data is redundant, just to make the code
  // better in shape.)
  int index;

  OperationPointInfo() {
    OperationPointInfo(-1);
  }

  OperationPointInfo(int index) : index(index) {
    shelf_id = -1;
    scheduled_to_change = false;
  }

  bool CapableForMoveOut() const {
    return shelf_id >= 0 && !scheduled_to_change;
  };

  bool CapableForMoveIn() const {
    return shelf_id < 0 && !scheduled_to_change;
  };
};

struct StoragePointInfo {
  // The shelf in this storage point. -1 for empty.
  int shelf_id;
  bool scheduled_to_change;
  // Index for this storage point. For both the StoragePointInfo array and
  // the storage point location array. (This data is redundant, just to make the code
  // better in shape.)
  int index;

  StoragePointInfo() {
    StoragePointInfo(-1);
  }

  StoragePointInfo(int index) : index(index) {
    shelf_id = -1;
    scheduled_to_change = false;
  }

  bool CapableForMoveOut() const {
    return shelf_id >= 0 && !scheduled_to_change;
  };

  bool CapableForMoveIn() const {
    return shelf_id < 0 && !scheduled_to_change;
  };
};

class KsWms : public KsWmsApi {
 public:
  KsWms(const KsMap& ks_map) :
      shelf_operation_points_(ks_map.GetShelfOperationPoints()),
      shelf_storage_points_(ks_map.GetShelfStoragePoints()),
      ks_map_(ks_map) {};
  void Init(KsSchedulerApi* scheduler_p);
  // Thread 1.
  void Run();

  // The mission status is reported in two steps.
  void ReportMissionStatus(MissionReport r) override;

 private:
  void ProcessReports();
  void GenOpToSpMissions();
  void GenSpToOpMissions();

  // Helpers.
  const WmsMission& GetPendingMission(int id);

  const KsMap& ks_map_;
  // Data.
  KsSchedulerApi* scheduler_p_;

  // Constant data structures.
  const std::vector<Location> shelf_operation_points_;
  const std::vector<Location> shelf_storage_points_;

  // A counter to generate mission id. This field is used only by the wms
  // thread, so no need to be protected by lock.
  int mission_id_counter_;

  // Data below is protected by mutex_.
  std::mutex mutex_;
  std::set<WmsMission> pending_missions_;
  int move_to_op_mission_count_;

  std::vector<StoragePointInfo> storage_point_info_;
  std::vector<OperationPointInfo> operation_point_info_;

  std::vector<ShelfInfo> shelf_info_;

  std::mutex mutex_io_;
  std::queue<MissionReport> message_queue_;
};

}

#endif