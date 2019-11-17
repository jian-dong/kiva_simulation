#ifndef KIVA_SIMULATION_SRC_COMMON_TYPES_H_
#define KIVA_SIMULATION_SRC_COMMON_TYPES_H_

#include <string>
#include <cassert>
#include <chrono>
#include <cmath>
#include <vector>
#include <map>

namespace ks {
using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

struct Location {
  int x, y;

  Location() { x = -1; y = -1; };
  Location(int x, int y) : x(x), y(y) {};

  Location &operator=(const Location &o) {
    x = o.x;
    y = o.y;
  }

  bool operator==(const Location &o) const {
    return x == o.x && y == o.y;
  }

  bool operator!=(const Location &o) const {
    return x != o.x || y != o.y;
  }

  bool operator<(const Location &o) const {
    if (x == o.x) {
      return y < o.y;
    } else {
      return x < o.x;
    }
  }

  std::string to_string() {
    return std::to_string(x) + " " + std::to_string(y);
  }
};

inline Location operator+(const Location& p, const std::pair<int, int>& o) {
  return Location(p.x + o.first, p.y + o.second);
}

inline Location operator-(const Location& p, const std::pair<int, int>& o) {
  return Location(p.x - o.first, p.y - o.second);
}

enum class Action : int {
  MOVE = 0,   // Move forward.
  CTURN = 1,  // Clockwise turn.
  CCTURN = 2, // Counter-clockwise turn.
  ATTACH = 3, // Attach to a shelf.
  DETACH = 4, // Detach the shelf.
  YIELD = 5,  // Wait At the current location, may not be used by certain types of planner.
};

// For debug use.
const std::map<Action, std::string> kActionToString =
    {{Action::MOVE, "MOVE"},
     {Action::CTURN, "CTURN"},
     {Action::CCTURN, "CCTURN"},
     {Action::ATTACH, "ATTACH"},
     {Action::DETACH, "DETACH"},
     {Action::YIELD, "YIELD"},};

enum class Direction : int {
  NORTH = 0,
  SOUTH = 1,
  WEST = 2,
  EAST = 3,
};

// For debug use.
const std::map<Direction, std::string> kDirectionToString =
    {{Direction::NORTH, "NORTH"},
     {Direction::SOUTH, "SOUTH"},
     {Direction::WEST, "WEST"},
     {Direction::EAST, "EAST"}};


inline Direction ClockwiseTurn(Direction d) {
  switch (d) {
    case Direction::NORTH:
      return Direction::EAST;
    case Direction::SOUTH:
      return Direction::WEST;
    case Direction::WEST:
      return Direction::NORTH;
    case Direction::EAST:
      return Direction::SOUTH;
    default:
      exit(0);
  }
}

inline Direction CounterClockwiseTurn(Direction d) {
  switch (d) {
    case Direction::NORTH:
      return Direction::WEST;
    case Direction::SOUTH:
      return Direction::EAST;
    case Direction::WEST:
      return Direction::SOUTH;
    case Direction::EAST:
      return Direction::NORTH;
    default:
      exit(0);
  }
}
}

#endif