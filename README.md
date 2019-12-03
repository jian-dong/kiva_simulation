# Kiva Simulation

## Implementation Notes

1. This program is based on a few papers: "SIPP: Safe Interval Path Planning for Dynamic Environments",
 "Complete decentralized method for on-line multi-robot trajectory planning in well-formed infrastructures",
 "Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks",
 "Persistent and Robust Execution of MAPF Schedules in Warehouses".

2. In this project communication between modules is written
as function call(receiver maintains a message queue with lock),
as compare to using RPC systems like Stubby. The advantage of this
approach is lighter weight, the disadvantage is having this non-standard
setup needs more reasoning.

3. Currently the program is coupled. Assume modules communicate with
message over internet may make the program structure cleaner.

4. For all message queue, the sender can send in an async way, the receiver should copy and process.

5. Computing the action dependency graph is time consuming(for 300 robots, it takes around 10s). Move it out of
the critical path makes the program more responsive.

6. Version 93b7c4278a745797b702530cb598e2c73950b442 is tested to 2000 tasks in debug mode.

## Lessons Learned

1. A large program must be able to be tested separately. To achieve this APIs
between components needs to be defined. There needs to be an API class and an implementation class.

2. The program shows strange behavior with -O2 turned on, should avoid -O2.

3. Should finish all behavior before turn on -O2, since with optimization turned on, it is much harder to debug.