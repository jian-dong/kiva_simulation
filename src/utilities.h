#ifndef KIVA_SIMULATION_SRC_UTILITIES_H_
#define KIVA_SIMULATION_SRC_UTILITIES_H_

#include <cstddef>
#include <thread>
#include <cstdlib>
#include <cmath>

#include "common_types.h"

namespace ks {

extern TimePoint (*GetCurrentTime)(void);

inline int GetManhattanDist(Location a, Location b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}

inline void SleepMS(size_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Random utilities.
// Return true with probability p%.
// 100 for always true, 0 for never true.
inline bool GetTrueWithProb(int p) {
  return rand() % 100 < p;
}

// Return true with probability a/b.
inline bool GetTrueWithProb(double a, double b) {
  return rand() < (a / b * RAND_MAX);
}

// Generate a random integer between lower bound @lb and upper bound @ub.
inline int GenRandomNumber(int lb, int ub) {
  return lb + ((double)rand() / (double)RAND_MAX) * (ub - lb);
}

inline bool ElapsedTimeLongerThanMs(TimePoint s, uint64_t duration_ms) {
  return GetCurrentTime() - s > std::chrono::milliseconds(duration_ms);
}

class Random {
  // TODO:
  // Currently the program is using rand() from cstdlib.
  // Calling it from multiple thread gives undefined behavior(but the program
  // won't crash, I guess).
  // This is not a problem for now. But there should be a better random device.
};

const double EPS = 1e-6;
// Compare two doubles with epsilon.
inline int DoubleEquals(double a, double b) {
  return fabs(a - b) < EPS;
}



}
#endif