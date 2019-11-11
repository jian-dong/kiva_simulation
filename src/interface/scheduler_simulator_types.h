#ifndef KIVA_SIMULATION_SRC_INTERFACE_SCHEDULER_SIMULATOR_TYPES_H_
#define KIVA_SIMULATION_SRC_INTERFACE_SCHEDULER_SIMULATOR_TYPES_H_

namespace ks {
// Scheduler to robot/simulator.
struct Command {
  int robot_id;
  std::vector<Action> actions;
};

// Robot/simulator to scheduler.
struct CommandReport {
  int robot_id;
  // The action that the robot just finished.
  Action action;

  CommandReport(int robot_id, Action action) : robot_id(robot_id), action(action) {};
};

}
#endif