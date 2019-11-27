#include "sipp_solver.h"

#include "common_types.h"
#include "ks_map.h"
#include "gtest/gtest.h"
#include "ks_scheduler_common.h"

using namespace std;
using namespace ks;

namespace {
class Basic : public ::testing::Test {
 protected:
  Basic() {
  }

  ~Basic() override {
  }

  void SetUp() override {
  }

  void TearDown() override {
  }
};

//TEST_F(Basic, Basic0) {
//  KsMap ks_map("../data/test_map_1.map");
//  ShelfManager shelf_manager;
//  SippSolver solver(ks_map);
//
//  RobotInfo robot(0, {0, 0});
//  robot.has_mission = true;
//  robot.mission.is_internal = true;
//  robot.mission.internal_mission.to = {0, 10};
//  PfRequest req;
//  req.robots.push_back(robot);
//  PfResponse resp = solver.FindPath(req, &shelf_manager);
//
//  for (ActionWithTimeSeq ac : resp.plan) {
//    for (ActionWithTime a : ac) {
//      cout << "action: " + kActionToString.at(a.action)
//           << " start: " << a.start_time_ms << " end: " << a.end_time_ms << endl;
//    }
//  }
//}
//
//
//TEST_F(Basic, Basic1) {
//  KsMap ks_map(kMapFilePath);
//  ShelfManager shelf_manager;
//  for (int i = 0; i < 10; i++) {
//    shelf_manager.AddMapping(i, {1, i + 7});
//  }
//  SippSolver solver(ks_map);
//
//  RobotInfo robot0(0, {0, 0});
//  robot0.has_mission = true;
//  robot0.shelf_attached = false;
//  robot0.mission.is_internal = true;
//  robot0.mission.internal_mission.to = {0, 10};
//  RobotInfo robot1(1, {0, 11});
//  robot1.has_mission = true;
//  robot1.shelf_attached = false;
//  robot1.mission.is_internal = true;
//  robot1.mission.internal_mission.to = {0, 0};
//
//  PfRequest req;
//  req.robots.push_back(robot0);
//  req.robots.push_back(robot1);
//  PfResponse resp = solver.FindPath(req, &shelf_manager);
//
//  for (int i = 0; i < resp.plan.size(); i++) {
//    ActionWithTimeSeq &ac = resp.plan[i];
//    cout << "robot id: " << i << endl;
//    for (ActionWithTime a : ac) {
//      cout << "action: " + kActionToString.at(a.action)
//           << " start: " << a.start_time_ms << " end: " << a.end_time_ms << endl;
//    }
//    cout << endl;
//  }
//}
//
//TEST_F(Basic, Basic1_WmsMission_WithShelf) {
//  KsMap ks_map(kMapFilePath);
//  ShelfManager shelf_manager;
//  for (int i = 0; i < 10; i++) {
//    shelf_manager.AddMapping(i, {1, i + 7});
//  }
//  SippSolver solver(ks_map);
//
//  RobotInfo robot0(0, {0, 0});
//  robot0.has_mission = true;
//  robot0.shelf_attached = true;
//  robot0.mission.is_internal = false;
//  robot0.mission.wms_mission.drop_to.loc = {0, 10};
//  RobotInfo robot1(1, {0, 11});
//  robot1.has_mission = true;
//  robot1.shelf_attached = true;
//  robot1.mission.is_internal = false;
//  robot1.mission.wms_mission.drop_to.loc = {0, 0};
//
//  PfRequest req;
//  req.robots.push_back(robot0);
//  req.robots.push_back(robot1);
//  PfResponse resp = solver.FindPath(req, &shelf_manager);
//
//  for (int i = 0; i < resp.plan.size(); i++) {
//    ActionWithTimeSeq &ac = resp.plan[i];
//    cout << "robot id: " << i << endl;
//    for (ActionWithTime a : ac) {
//      cout << "action: " + kActionToString.at(a.action)
//           << " start: " << a.start_time_ms << " end: " << a.end_time_ms << endl;
//    }
//    cout << endl;
//  }
//}
//
//
//TEST_F(Basic, Basic2) {
//  KsMap ks_map(kMapFilePath);
//  ShelfManager shelf_manager;
//  SippSolver solver(ks_map);
//
//  RobotInfo robot0(0, {0, 0});
//  robot0.pos.dir = Direction::EAST;
//  robot0.has_mission = true;
//  robot0.mission.is_internal = true;
//  robot0.mission.internal_mission.to = {0, 4};
//  RobotInfo robot1(1, {0, 5});
//  robot1.pos.dir = Direction::WEST;
//  robot1.has_mission = true;
//  robot1.mission.is_internal = true;
//  robot1.mission.internal_mission.to = {0, 0};
//
//  PfRequest req;
//  req.robots.push_back(robot0);
//  req.robots.push_back(robot1);
//  PfResponse resp = solver.FindPath(req, &shelf_manager);
//
//  for (int i = 0; i < resp.plan.size(); i++) {
//    ActionWithTimeSeq &ac = resp.plan[i];
//    cout << "robot id: " << i << endl;
//    for (ActionWithTime a : ac) {
//      cout << "action: " + kActionToString.at(a.action)
//           << " start: " << a.start_time_ms << " end: " << a.end_time_ms << endl;
//    }
//    cout << endl;
//  }
//}

//TEST_F(Basic, Basic3) {
//  KsMap ks_map(kMapFilePath);
//  ShelfManager shelf_manager;
//  for (int i = 0; i< ks_map.shelf_count_; i++) {
//    shelf_manager.AddMapping(0, ks_map.GetShelfStoragePoints()[i]);
//  }
//
//  SippSolver solver(ks_map);
//
//  RobotInfo robot0(0, {1, 7});
//  robot0.pos.dir = Direction::NORTH;
//  robot0.has_mission = true;
//  robot0.mission.is_internal = false;
//  robot0.mission.wms_mission.pick_from.loc = {1, 47};
//  robot0.mission.wms_mission.drop_to.loc = {79, 95};
//
//  RobotInfo robot1(0, {1, 8});
//  robot1.pos.dir = Direction::NORTH;
//  robot1.has_mission = true;
//  robot1.mission.is_internal = false;
//  robot1.mission.wms_mission.pick_from.loc = {3, 37};
//  robot1.mission.wms_mission.drop_to.loc = {26, 2};
//
//  PfRequest req;
//  req.robots.push_back(robot0);
//  req.robots.push_back(robot1);
//  PfResponse resp = solver.FindPath(req, &shelf_manager);
//
//  for (int i = 0; i < resp.plan.size(); i++) {
//    ActionWithTimeSeq &ac = resp.plan[i];
//    cout << "robot id: " << i << endl;
//    for (ActionWithTime a : ac) {
//      cout << "action: " + kActionToString.at(a.action)
//           << " start: " << a.start_time_ms << " end: " << a.end_time_ms << endl;
//    }
//    cout << endl;
//  }
//}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
