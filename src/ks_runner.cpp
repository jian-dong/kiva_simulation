#include "ks_runner.h"

#include <cstdlib>
#include <ctime>

namespace ks {

using namespace std;

void KsRunner::Run() {
  // Start a thread for wms.

}

void KsRunner::Init() {
  srand(time(NULL));

  // Create stubs.
  map_p_ = new KsMap(kMapFileName);
//  simulator_p_ = new KsSimulator();
//  scheduler_p_ = new KsScheduler(*map_p_);
//  wms_p_ = new KsWms(*map_p_);

  // Wire stubs up.
//  simulator_p_->Init(scheduler_p_, *map_p_);
//  scheduler_p_->Init(wms_p_, simulator_p_);
//  wms_p_->Init(scheduler_p_);
//  wms_p_->Init(nullptr);
}

}