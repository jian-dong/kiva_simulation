// A mock scheduler to test/debug simulator.
#include <iostream>
#include <list>

#include "interface/ks_api.h"
#include "ks_map.h"
#include "ks_simulator.h"
#include "utilities.h"

namespace ks {

using namespace std;

class MockScheduler : public KsSchedulerApi {
 public:
  MockScheduler() = default;
  void Init(KsSimulatorApi* simulator_p);
  void Run();
  void AddMission(WmsMission mission) override;
  void ReportActionDone(CommandReport r) override;
 private:
  std::mutex mutex_;
  int pending_actions_;
  list<Action> action_seq;
  KsSimulatorApi* simulator_p_;
};

void MockScheduler::Init(KsSimulatorApi* simulator_p) {
  simulator_p_ = simulator_p;
  pending_actions_ = 0;
  action_seq.push_back(Action::CTURN);
  for (int j = 0; j < 5; j++) {
    action_seq.push_back(Action::MOVE);
  }
  action_seq.push_back(Action::ATTACH);
  for (int j = 0; j < 5; j++) {
    action_seq.push_back(Action::MOVE);
  }
  action_seq.push_back(Action::CTURN);
  for (int j = 0; j < 2; j++) {
    action_seq.push_back(Action::MOVE);
  }
  action_seq.push_back(Action::CCTURN);
  for (int j = 0; j < 5; j++) {
    action_seq.push_back(Action::MOVE);
  }
  action_seq.push_back(Action::DETACH);
}

void MockScheduler::Run() {
  // Should send a sequence of commands.
  while (!action_seq.empty()) {
    mutex_.lock();
    if (pending_actions_ > 0) {
      mutex_.unlock();
      SleepMS(100);
      continue;
    }
    pending_actions_++;
    mutex_.unlock();

    Action cur_action = action_seq.front();
    cout << "Sending out action: " << kActionToString.at(cur_action) << endl;
    simulator_p_->AddActions({0, {cur_action}});
  }

  while (true) {

  }
}

void MockScheduler::AddMission(WmsMission mission) {
  // Do nothing, will create missions manually.
}

void MockScheduler::ReportActionDone(CommandReport r) {
  lock_guard lock(mutex_);
  assert(action_seq.front() == r.action);
  action_seq.pop_front();
  pending_actions_--;
}
}

using namespace ks;

int main() {
  auto* map_p = new KsMap(kMapFilePath);
  auto* mock_scheduler = new MockScheduler();
  auto* simulator = new KsSimulator();

  // Wire stubs up.
  simulator->Init(mock_scheduler, *map_p);
  mock_scheduler->Init(simulator);

  std::thread t0(&KsSimulator::Run, simulator);
  t0.detach();

  mock_scheduler->Run();
  return 0;
}