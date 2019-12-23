# Kiva Simulation

## Introduction

此程序实现了以下文章：
* "SIPP: Safe Interval Path Planning for Dynamic Environments",
* "Complete decentralized method for on-line multi-robot trajectory planning in well-formed infrastructures",
* "Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks",
* "Persistent and Robust Execution of MAPF Schedules in Warehouses".

此程序的运行效果可参考视频：
* v.youku.com/v_show/id_XNDQ2MTgzNzczMg==.html

## To Run

1. Compile. In the base directory of this project, do "mkdir build && cd build && cmake .. && make".
2. Make sure redis server is started, and there is no key named "ks".
With Ubuntu, to start redis, run "redis-server", then do "echo 'del ks' | redis-cli".
3. Start the python GUI by "python3 ../src/manual_utilities/ks_gui.py". If the module "tk" was not installed, try pip3.
4. Start the main program by "./bin/local_test_main".

## Implementation Notes

1. In this project, communication between modules is written
as function call(receiver maintains a message queue with lock),
as compared to use RPC systems like Stubby. The advantage of this
approach is lighter weight, the disadvantage is having this non-standard
setup needs more reasoning.

2. Currently the program is coupled. Assume modules communicate with
message over internet may make the program structure cleaner.

3. For all message queue, the sender can send in an async way, the receiver should copy and process.

4. Computing the action dependency graph is time consuming(for 300 robots, it takes around 10s). Move it out of
the critical path makes the program more responsive.

## Lessons Learned

1. A large program must be able to be tested separately. To achieve this APIs
between components needs to be defined. There needs to be an API class and an implementation class.

2. Should finish all behavior before turn on -O2, since with optimization turned on, it is much harder to debug.