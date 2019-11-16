#ifndef KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_COMMON_H_
#define KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_COMMON_H_

#include "common_types.h"
#include "logger.h"
#include "constants.h"

namespace ks {
inline double GetActionCostInTime(Action a) {
  if (a == Action::ATTACH || a == Action::DETACH) {
    return ((double) kAttachDetachDurationMs) / 1000;
  }
  if (a == Action::CCTURN || a == Action::CTURN) {
    return ((double) kTurnDurationMs) / 1000;
  }
  if (a == Action::MOVE) {
    return ((double) kMoveDurationMs) / 1000;
  }
  if (a == Action::YIELD) {
    LogFatal("Invalid action for SIPP.");
  }
}
}
#endif