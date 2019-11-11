#Kiva Simulation

##Implementation Notes
1. In this project communication between modules is written
as function call(receiver maintains a message queue with lock),
as compare to using RPC systems like Stubby. The advantage of this
approach is lighter weight, the disadvantage is having this non-standard
setup needs more reasoning.

2. Currently the program is coupled. Assume modules communicate with
message over internet may make the program structure cleaner.