#ifndef KIVA_SIMULATION_SRC_KS_SCHEDULER_API_H_
#define KIVA_SIMULATION_SRC_KS_SCHEDULER_API_H_

#include "wms_scheduler_types.h"
#include "scheduler_simulator_types.h"

namespace ks {
class KsWmsApi {
 public:
  // The mission status is reported in two steps.
  virtual void ReportMissionStatus(MissionReport r) = 0;
};

class KsSchedulerApi {
 public:
  virtual void AddMission(WmsMission mission) = 0;
  virtual void ReportActionDone(CommandReport r) = 0;
};

class KsSimulatorApi {
 public:
  virtual void AddActions(Command command) = 0;
};


}

#endif