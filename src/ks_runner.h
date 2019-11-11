#ifndef KIVA_SIMULATION_SRC_KS_RUNNER_H_
#define KIVA_SIMULATION_SRC_KS_RUNNER_H_

#include "ks_map.h"
#include "ks_scheduler.h"
#include "ks_simulator.h"
#include "ks_wms.cpp"

namespace ks {

class KsRunner {
 public:
  KsRunner() = default;
  void Init();
  void Run();

 private:
  KsMap* map_p_;
  KsWms* wms_p_;
  KsScheduler* scheduler_p_;
  KsSimulator* simulator_p_;
};

}

#endif