#include "ks_runner.h"

#include <cstdlib>
#include <ctime>
#include <thread>

namespace ks {

void KsRunner::Start() {
  srand(time(NULL));

  // Create stubs.
  map_p_ = new KsMap(kMapFileName);
  KsSimulator* simulator = new KsSimulator();
  simulator_p_ = simulator;
  KsScheduler* scheduler = new KsScheduler(*map_p_);
  scheduler_p_ = scheduler;
  KsWms* wms = new KsWms(*map_p_);
  wms_p_ = wms;

  // Wire stubs up.
  simulator->Init(scheduler_p_, *map_p_);
  scheduler->Init(wms_p_, simulator_p_);
  wms->Init(scheduler_p_);

  // Run.
  std::thread t0(&KsSimulator::Run, simulator);
  t0.detach();

  std::thread t1(&KsScheduler::Run, scheduler);
  t1.detach();

  std::thread t2(&KsScheduler::AdgRunner, scheduler);
  t2.detach();

  wms->Run();
}

}