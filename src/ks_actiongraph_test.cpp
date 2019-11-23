#include "ks_actiongraph.h"

#include "gtest/gtest.h"

using namespace std;
using namespace ks;

namespace {

void PrintCommands(const vector<vector<Action>> &commands) {
  int robot_count = commands.size();
  for (int i = 0; i < robot_count; i++) {
    cout << "robot " << i << ": ";
    for (Action a : commands[i]) {
      cout << kActionToString.at(a) << " ";
    }
    cout << endl;
  }
}

void PrintRobotInfo(const vector<RobotInfo> &robot_info) {
  int robot_count = robot_info.size();
  for (int i = 0; i < robot_count; i++) {
    cout << robot_info[i].to_string() << endl;
  }
}

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

TEST_F(Basic, Basic0) {
  const int robot_count = 2;
  KsActionGraph ag(robot_count);
  vector<ActionWithTimeSeq> plan(robot_count);
  plan[0].push_back({Action::MOVE,
                     0,
                     1000,
                     {0, 0, Direction::EAST},
                     {0, 1, Direction::EAST}});
  plan[0].push_back({Action::MOVE,
                     1000,
                     2000,
                     {0, 1, Direction::EAST},
                     {0, 2, Direction::EAST}});
  plan[1].push_back({Action::MOVE,
                     2500,
                     3500,
                     {1, 1, Direction::NORTH},
                     {0, 1, Direction::NORTH}});

  ag.SetPlan(plan);

  PrintCommands(ag.GetCommands());
  ag.UpdateRobotStatus(0, Action::MOVE);
  PrintCommands(ag.GetCommands());
  ag.UpdateRobotStatus(0, Action::MOVE);
  PrintCommands(ag.GetCommands());
  ag.UpdateRobotStatus(1, Action::MOVE);
  PrintCommands(ag.GetCommands());
}

TEST_F(Basic, Cut) {
  const int robot_count = 2;
  KsActionGraph ag(robot_count);
  vector<ActionWithTimeSeq> plan(robot_count);
  plan[0].push_back({Action::MOVE,
                     0,
                     1000,
                     {0, 0, Direction::EAST},
                     {0, 1, Direction::EAST}});
  plan[0].push_back({Action::MOVE,
                     1000,
                     2000,
                     {0, 1, Direction::EAST},
                     {0, 2, Direction::EAST}});
  plan[1].push_back({Action::MOVE,
                     2500,
                     3500,
                     {1, 1, Direction::NORTH},
                     {0, 1, Direction::NORTH}});

  ag.SetPlan(plan);

  PrintCommands(ag.GetCommands());
  ag.UpdateRobotStatus(0, Action::MOVE);

  vector<RobotInfo> robot_info;
  robot_info.emplace_back(0, Location(0, 0));
  robot_info.emplace_back(1, Location(0, 0));
  robot_info[0].pos = {0, 1, Direction::EAST};
  robot_info[1].pos = {1, 1, Direction::NORTH};
  ag.Cut(robot_info);
  PrintRobotInfo(robot_info);
  vector<ActionWithTimeSeq> new_plan(robot_count);
  new_plan[1].push_back({Action::MOVE,
                     2500,
                     3500,
                     {1, 1, Direction::NORTH},
                     {0, 1, Direction::NORTH}});
  ag.SetPlan(new_plan);
  // This GetCommands should return nothing.
  PrintCommands(ag.GetCommands());
  ag.UpdateRobotStatus(0, Action::MOVE);

  // Switched to new plan.
  PrintCommands(ag.GetCommands());
  ag.UpdateRobotStatus(1, Action::MOVE);
  // Finished new plan, returns nothing.
  PrintCommands(ag.GetCommands());
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
