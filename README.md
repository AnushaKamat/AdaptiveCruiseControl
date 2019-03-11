Adaptive Cruise Control based on elma
===
Scenario
---
Adaptive Cruise Control(ACC) would use the elma library to create a simulation of driving scenario. It would have a car with Adaptive Cruise Control where in the driver of the car can set the speed and safety distance for ACC. Once these parameters are set, the car runs with the set speed. In case , the driver presses the brake then the ACC gets switched off and is over ridden manually.When ACC is in operation and another car comes in the safe distance set by the driver then it tries to decrease the speed to maintain the distance from the car infront of it.

Implementation Details
---
- Car would be inherited by a process class

- Cruise Control would be inherited by a process class

- Adaptive Cruise Control would be inherited either by Cruise Control/Process(Design choice)
    It will set Adaptive Cruise control speed depending on 
    - speed set by driver
    - another car in the safety distance

- Driver would be inherited by process class with below operations in scope
    - set speed
    - set safety distance
    - press brake to override ACC

Milestones 
---
1. Talk to Professor about the project and it's feasibility. Date : 01/09 -DONE
2. Create ACC class and share events between driver, car and ACC. Date : 01/13
3. Create driving environment where another interacts with car enabled with ACC. Date : 01/17
4. Write the test cases and get the testing done. Date : 01/20
5. Submit documentation with tests. Date : 01/22

Installation
---

The source code for Elma [is on Github](https://github.com/klavinslab/elma).



References
---
The example of Adaptive Cruise Control is mainly built on [elma](https://github.com/klavinslab/elma) library using following references:
- [gcc](https://gcc.gnu.org/)
- [Doxygen](http://www.doxygen.nl/)
- [Google Test](https://github.com/google/googletest)
- Neils Lohmann's JSON library: https://github.com/nlohmann/json
- [yhirose](https://github.com/yhirose)'s HTTP library
- Docker Image [klavins/ecep520](https://hub.docker.com/r/klavins/ecep520)

Usage
---
See the examples in the `examples` directory.

License
---
This software is open source and uses the MIT license. Details can be found [here](https://github.com/klavinslab/elma).
