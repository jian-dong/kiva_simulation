#include "ks_map.h"

#include <fstream>
#include <iostream>
#include <vector>

#include "logger.h"

namespace ks {
using namespace std;

KsMap::KsMap(std::string file_name) {
  ifstream input(file_name);
  if (!input) {
    cout << "Cannot open map file" << endl;
    exit(0);
  }

  vector<string> buffer;
  string tmp_line;
  while (getline(input, tmp_line)) {
    if (tmp_line[0] == '#') {
      // Skip comments.
      continue;
    }
    buffer.push_back(tmp_line);
  }
  input.close();

  for (int x = 0; x < kXLimit; x++) {
    for (int y = 0; y < kYLimit; y++) {
      map_[x][y] = buffer[x][y];

      if (map_[x][y] == kShelfOperationPoint) {
        sops_.emplace_back(x, y);
      }
      if (map_[x][y] == kShelfStoragePoint) {
        ssps_.emplace_back(x, y);
      }
      if (map_[x][y] != 'B' && map_[x][y] != 'O' && map_[x][y] != 'T') {
        passable_.emplace_back(x, y);
      }
      if (map_[x][y] == kRestArea) {
        rest_area_.emplace_back(x, y);
      }
    }
  }
}

const std::vector<Location> &KsMap::GetShelfOperationPoints() const {
  return sops_;
}

const std::vector<Location> &KsMap::GetShelfStoragePoints() const {
  return ssps_;
}

const std::vector<Location> &KsMap::GetPassableLocations() const {
  return passable_;
}

const std::vector<Location> &KsMap::GetRestAreas() const {
  return rest_area_;
}

bool KsMap::IsLocationPassable(const Location &loc) const {
  int x = loc.x, y = loc.y;
  if (x < 0 || x >= kXLimit || y < 0 || y >= kYLimit) {
    return false;
  }
  return map_[x][y] != 'B' && map_[x][y] != 'O' && map_[x][y] != 'T';
}

}