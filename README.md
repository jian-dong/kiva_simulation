# Kiva Simulation

## Implementation Notes

1. In this project communication between modules is written
as function call(receiver maintains a message queue with lock),
as compare to using RPC systems like Stubby. The advantage of this
approach is lighter weight, the disadvantage is having this non-standard
setup needs more reasoning.

2. Currently the program is coupled. Assume modules communicate with
message over internet may make the program structure cleaner.

## Lessons Learned

1. A large program must be able to be tested separately. To achieve this APIs
between components needs to be defined. There needs to be an API class and an implementation class.