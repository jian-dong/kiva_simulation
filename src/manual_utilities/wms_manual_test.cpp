// A mock scheduler to test/debug wms.
#include <list>
#include <queue>
#include <mutex>
#include <iostream>

#include "interface/ks_api.h"
#include "utilities.h"
#include "ks_scheduler_common.h"
#include "ks_map.h"
#include "ks_wms.h"

namespace ks {

using namespace std;

struct MissionProgress {
  WmsMission m;
  bool half_finished;

  MissionProgress(WmsMission m) : m(m), half_finished(false) {}
};

class MockScheduler : public KsSchedulerApi {
 public:
  MockScheduler(const KsMap& ks_map) : ks_map_(ks_map) {};
  void Init(KsWmsApi *wms);
  void Run();
  void AddMission(WmsMission mission) override;
  void ReportActionDone(CommandReport r) override;
 private:
  const KsMap& ks_map_;
  mutex mutex_io_;
  vector<MissionProgress> missions_;
  queue<WmsMission> incoming_missions_;
  KsWmsApi* wms_;
  ShelfManager shelf_manager_;
};

void MockScheduler::Run() {
  while (true) {
    mutex_io_.lock();
    while (!incoming_missions_.empty()) {
      missions_.emplace_back(incoming_missions_.front());
      incoming_missions_.pop();
    }
    mutex_io_.unlock();
    if (!missions_.empty()) {
      ShuffleVector(missions_);
      MissionProgress& m = missions_.back();
      if (!m.half_finished) {
        // Finished the first half of the mission.
        cout << "Mission: " << to_string(m.m.id) << " pickup done. from: " << m.m.pick_from.loc.to_string() << endl;
        m.half_finished = true;
        shelf_manager_.RemoveMapping(m.m.shelf_id, m.m.pick_from.loc);
        wms_->ReportMissionStatus({m.m.id, MissionReportType::PICKUP_DONE});
      } else {
        // Finished the second half of the mission.
        missions_.pop_back();
        cout << "Mission: " << to_string(m.m.id) << " done. to: " << m.m.drop_to.loc.to_string() << endl;
        shelf_manager_.AddMapping(m.m.shelf_id, m.m.drop_to.loc);
        wms_->ReportMissionStatus({m.m.id, MissionReportType::MISSION_DONE});
      }
    }
    SleepMS(GenRandomNumber(1000, 1500));
  }
}

void MockScheduler::AddMission(WmsMission mission) {
  lock_guard<mutex> lock(mutex_io_);
  cout << "received mission: from: " << mission.pick_from.loc.to_string()
      << " to: " << mission.drop_to.loc.to_string()
      << " shelf id: " << mission.shelf_id << endl;
  incoming_missions_.push(mission);
}

void MockScheduler::ReportActionDone(CommandReport r) {
}

void MockScheduler::Init(KsWmsApi *wms) {
  wms_ = wms;
  const std::vector<Location>& shelf_storage_points = ks_map_.GetShelfStoragePoints();

  // The way shelves are initialized.
  for (int i = 0; i < ks_map_.actual_shelf_count_; i++) {
    shelf_manager_.AddMapping(i, shelf_storage_points[i]);
  }
}
}

using namespace ks;

int main() {
  auto* map_p = new KsMap(kMapFilePath);
  auto* mock_scheduler_p = new MockScheduler(*map_p);
  KsWms* wms_p = new KsWms(*map_p);

  // Wire stubs up.
  mock_scheduler_p->Init(wms_p);
  wms_p->Init(mock_scheduler_p);

  std::thread t0(&MockScheduler::Run, mock_scheduler_p);
  t0.detach();

  wms_p->Run();

  return 0;
}