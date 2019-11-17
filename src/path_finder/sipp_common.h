#ifndef KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_COMMON_H_
#define KIVA_SIMULATION_SRC_PATH_FINDER_SIPP_COMMON_H_

#include "common_types.h"
#include "logger.h"
#include "constants.h"

namespace ks {
inline int GetActionCostInTime(Action a) {
  if (a == Action::ATTACH || a == Action::DETACH) {
    return kAttachDetachDurationMs;
  }
  if (a == Action::CCTURN || a == Action::CTURN) {
    return kTurnDurationMs;
  }
  if (a == Action::MOVE) {
    return kMoveDurationMs;
  }
  if (a == Action::YIELD) {
    LogFatal("Invalid action for SIPP.");
  }
}
}
#endif