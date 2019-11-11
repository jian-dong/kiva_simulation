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

 private:
  char map_[kXLimit][kYLimit];

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