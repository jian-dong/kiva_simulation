#ifndef KIVA_SIMULATION_SRC_KS_MAP_H_
#define KIVA_SIMULATION_SRC_KS_MAP_H_

#include <string>
#include <vector>

#include "common_types.h"

namespace ks {

class KsMap {
 public:
  explicit KsMap(std::string file_name);
  [[nodiscard]] const std::vector<Location>& GetPassableLocations() const;
  [[nodiscard]] const std::vector<Location>& GetShelfOperationPoints() const;
  [[nodiscard]] const std::vector<Location>& GetShelfStoragePoints() const;
  [[nodiscard]] bool IsLocationPassable(const Location& loc) const;

  int robot_count_;
  int shelf_count_;
  int operation_point_count_;
  int storage_point_count_;

 private:
  const static int kXLimit = 120;
  const static int kYLimit = 120;

  // Data.
  int x_limit_, y_limit_;
  char map_[kXLimit][kYLimit];

  // Shelf operation points.
  std::vector<Location> sops_;
  // Shelf storage points.
  std::vector<Location> ssps_;
  // Passable loations.
  std::vector<Location> passable_;
};

}

#endif