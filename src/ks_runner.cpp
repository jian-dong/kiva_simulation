#include "ks_runner.h"

#include <cstdlib>
#include <ctime>
#include <thread>

#include "ks_scheduler.h"
#include "ks_simulator.h"
#include "ks_wms.h"

namespace ks {

void KsRunner::Start() {
//  srand(time(nullptr));
  // TODO: this may be useful for debug.
  srand(0);

  // Create stubs.
  map_p_ = new KsMap(kMapFilePath);
  KsSimulator* simulator_p = new KsSimulator();
  simulator_p_ = simulator_p;
  KsScheduler* scheduler_p = new KsScheduler(*map_p_);
  scheduler_p_ = scheduler_p;
  KsWms* wms_p = new KsWms(*map_p_);
  wms_p_ = wms_p;

  // Wire stubs up.
  simulator_p->Init(scheduler_p_, *map_p_);
  scheduler_p->Init(wms_p_, simulator_p_);
  wms_p->Init(scheduler_p_);

  // Run.
  std::thread t0(&KsSimulator::Run, simulator_p);
  t0.detach();

  std::thread t1(&KsScheduler::Run, scheduler_p);
  t1.detach();

  std::thread t2(&KsScheduler::AdgRunner, scheduler_p);
  t2.detach();

  wms_p->Run();
}

}