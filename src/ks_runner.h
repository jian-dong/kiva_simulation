#ifndef KIVA_SIMULATION_SRC_KS_RUNNER_H_
#define KIVA_SIMULATION_SRC_KS_RUNNER_H_

#include "interface/ks_api.h"
#include "ks_map.h"

namespace ks {

class KsRunner {
 public:
  KsRunner() = default;
  void Start();

 private:
  KsMap* map_p_;
  KsWmsApi* wms_p_;
  KsSchedulerApi* scheduler_p_;
  KsSimulatorApi* simulator_p_;
};

}

#endif