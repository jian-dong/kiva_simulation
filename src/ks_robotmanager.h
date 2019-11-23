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
  KsRobotManager(const KsMap &ks_map) : map_(ks_map), rest_areas_(ks_map.GetRestAreas()) {};
  void Init();

  void AssignMissions(std::set<WmsMission> &missions);
  std::optional<MissionReport> UpdateRobotStatus(int robot_id, Action a);
  const std::vector<RobotInfo>& GetRobotInfo() const { return robot_info_; };

 private:
  // Helper functions.
  std::vector<RobotInfo*> GetIdleRobots();
  RobotInfo* GetIdleRobotAtLocation(Location loc);
  RobotInfo* GetClosestIdleRobot(Location loc);
  bool HasIdleRobots() { return !GetIdleRobots().empty();};
  Location AssignToRestArea(int robot_id);
  void FreeFromRestArea(int robot_id);

  // Stubs.
  const KsMap &map_;
  const std::vector<Location>& rest_areas_;
  std::vector<int> rest_area_assignment_;

  // Data.
  std::vector<RobotInfo> robot_info_;
};
}

#endif