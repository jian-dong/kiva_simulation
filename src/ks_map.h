#ifndef KIVA_SIMULATION_SRC_KS_MAP_H_
#define KIVA_SIMULATION_SRC_KS_MAP_H_

#include <string>
#include <vector>

#include "common_types.h"
#include "constants.h"

namespace ks {

class KsMap {
 public:
  explicit KsMap(std::string file_name);
  const std::vector<Location>& GetPassableLocations() const;
  const std::vector<Location>& GetShelfOperationPoints() const;
  const std::vector<Location>& GetShelfStoragePoints() const;
  const std::vector<Location>& GetRestAreas() const;
  bool IsLocationPassable(const Location& loc) const;

  int actual_robot_count_;
  int actual_shelf_count_;

 private:
  char map_[kXLimit][kYLimit];
  int actual_x_limit_, actual_y_limit_;

  // Constants.
  // Shelf operation points.
  std::vector<Location> sops_;
  // Shelf storage points.
  std::vector<Location> ssps_;
  // Passable loations.
  std::vector<Location> passable_;
  std::vector<Location> rest_area_;
};

}

#endif