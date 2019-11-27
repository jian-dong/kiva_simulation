#ifndef KIVA_SIMULATION_SRC_KS_ROBOTMANAGER_H_
#define KIVA_SIMULATION_SRC_KS_ROBOTMANAGER_H_

#include <vector>
#include <set>
#include <optional>

#include "common_types.h"
#include "ks_map.h"
#include "ks_scheduler_common.h"
#include "interface/ks_api.h"

namespace ks {
// A pure data class.
class KsRobotManager {
 public:
  KsRobotManager(const KsMap &ks_map) : ks_map_(ks_map) {};
  void Init();

  void AssignMissions(std::set<WmsMission> &missions);
  std::optional<MissionReport> UpdateRobotStatus(int robot_id, Action a);
  const std::vector<RobotInfo>& GetRobotInfo() const { return robot_info_; };

 private:
  // Helper functions.
  std::vector<RobotInfo*> GetIdleRobots();
  RobotInfo* GetIdleRobotAtLocation(Location loc);
  RobotInfo* GetClosestIdleRobot(Location loc);
  bool HasIdleRobot() { return !GetIdleRobots().empty();};
  bool IsMissionValid(const WmsMission &mission);
  std::set<Location> GetFreeLocations(const std::set<WmsMission> &missions);

  // Stubs.
  const KsMap &ks_map_;
  // Data.
  std::vector<RobotInfo> robot_info_;
};
}

#endif