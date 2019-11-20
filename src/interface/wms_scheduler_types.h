#ifndef KIVA_SIMULATION_SRC_INTERFACE_WMS_SCHEDULER_TYPES_H_
#define KIVA_SIMULATION_SRC_INTERFACE_WMS_SCHEDULER_TYPES_H_

#include "common_types.h"

namespace ks {
// Scheduler to WMS.
enum class MissionReportType {
  PICKUP_DONE = 0,
  MISSION_DONE = 1,
};

struct MissionReport {
  int mission_id;
  MissionReportType type;

  MissionReport(int mission_id, MissionReportType type)
      : mission_id(mission_id), type(type) {};
};

// WMS to scheduler.
enum class LocationType : int {
  STORAGE_POINT = 0,
  OPERATION_POINT = 1,
};

struct LocationInfo {
  Location loc;
  LocationType type;
  int index;
};

struct WmsMission {
  int id;
  // A mission always contains two parts, pickup a shelf from the @from location,
  // then put it down at the @to location.
  LocationInfo pick_from, drop_to;

  // The fields below are not used by scheduler and modules below, these
  // are used by WMS.
  int shelf_id;

  WmsMission() = default;
  WmsMission(int id) : id(id) {};

  WmsMission &operator=(const WmsMission &o) = default;

  bool operator<(const WmsMission &o) const {
    return id < o.id;
  }

  std::string to_string() {
    return "from: " + pick_from.loc.to_string() +
        " to: " + drop_to.loc.to_string() +
        " shelf id: " + std::to_string(shelf_id);
  }
};
}
#endif