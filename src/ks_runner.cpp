#include "ks_runner.h"

#include <cstdlib>
#include <ctime>
#include <thread>

namespace ks {

void KsRunner::Start() {
  srand(time(NULL));

  // Create stubs.
  map_p_ = new KsMap(kMapFileName);
  simulator_p_ = new KsSimulator();
  scheduler_p_ = new KsScheduler(*map_p_);
  wms_p_ = new KsWms(*map_p_);

  // Wire stubs up.
  simulator_p_->Init(scheduler_p_, *map_p_);
  scheduler_p_->Init(wms_p_, simulator_p_);
  wms_p_->Init(scheduler_p_);

  // Run.
  std::thread t0(&KsSimulator::Run, simulator_p_);
  t0.detach();
//  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::thread t1(&KsScheduler::Run, scheduler_p_);
  t1.detach();

  std::thread t2(&KsScheduler::AdgRunner, scheduler_p_);
  t2.detach();

  wms_p_->Run();
}

}