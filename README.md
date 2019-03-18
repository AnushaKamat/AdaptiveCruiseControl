Adaptive Cruise Control based on elma
===
Scenario
---
Adaptive Cruise Control(ACC) would use the elma library to create a simulation of driving scenario. It would have a car with Adaptive Cruise Control where in the driver of the car can set the speed and safety distance for ACC. Once these parameters are set, the car runs with the set speed. In case , the driver presses the brake then the ACC gets switched off and is over ridden manually.When ACC is in operation and another car comes in the safe distance set by the driver then it tries to decrease the speed to maintain the distance from the car infront of it.

Implementation Details
---
- Car would be inherited by a process class

- Cruise Control would be inherited by a process class

- Adaptive Cruise Control would be inherited either by Cruise Control/Process(Design choice).

    It will set Adaptive Cruise control speed depending on 
    - speed set by driver
    - another car in the safety distance

- Driver would be inherited by process class with below operations in scope
    - set speed
    - set safety distance
    - press brake to override ACC

Design Details(Added on 3/17):
---
- Cruise Control is being redesigned.
    - Driver sets Cruise On/Off which is emitted as an event to CruiseControl class
    - When CC is switched on , the driver also sets the desired speed which is sent over the channel.
    - CruiseControl class has an handler for change in `CC_on`
    - CruiseControl class sets the value of throttle accordingly and sends it over channel to the Car class.
    - Car class, depending on the current velocity of the car and new throttle value, sets the new speed of the car.
- Adaptive Cruise Control is designed in the class `AutoCruiseControl`.
    - Driver switches on `ACC_on` switch to enable Adaptive Cruise Control to kick in.This triggers an event.
    - When ACC is switched on, Driver has to set desired speed and safety distance.
    - In AutoCruiseControl class, depending on `ACC status` event, the desired speed, safety distance and current velocity of Car is read in.
    - In addition to this, ACC is being catered to by another class called Radar inherited from Process.
        Radar sends the distance from the car to the next obstacle.
    - In ACC, if obstacle is farther than safety distance, the ACC keeps cruising at the same desired speed set by the driver.
    - If ACC finds that obstacle is in the range of safety distance set, in order to prevent damage, it decreases the desired speed by 5 till it     can maintain the safe distance from the obstacle. <br/>

    Assumptions: <br/>
    The obstacle is also contsantly moving wrt to car either at slow speed or higher speed than that of ACC car.



Milestones 
---
1. Talk to Professor about the project and it's feasibility. Date : 03/09 -DONE
2. Create ACC class and share events between driver, car and ACC. Date : 03/13<br/>
    PROGRESS :<br/>
        - 3/15 : Cruise Control is modified/ Simulated Time run is created<br/>
        - 3/16 : Different Classes for CC and ACC. Driver Class is modified to accomodate different driving styles.
3. Create driving environment where another interacts with car enabled with ACC. Date : 03/17<br/>
    PROGRESS : <br/>
        - 3/17 : Tested the driving scenarios using the environment.<br/>

    TODO : <br/>
        - Modularise ACC example with different classes in different files.<br/>
        - Adapt to ELMA project Folder structure. <br/>
        - Check the new pull for Simulated Run and compare with existing one. <br/>

4. Write the test cases and get the testing done. Date : 03/20
5. Submit documentation with tests. Date : 03/22

Strech Goals
---
1. Interaction of car enabled with ACC with a regular car.<br/>
    PROGRESS: Interaction of ACC enabled car with moving obstacle is met. Obstacle is not modelled as a Car.
2. Car with Camera function and not ACC. - which could read sign boards and change the speed of car dynamically till driver intervenes.
3. Simulation of Car with respect to ACC enabled Car.
4. Plot of distance covered by regular car vs the distance covered by ACC enabled car wrt time.(depicting its speed curve)
5. [3/17] Consider having a UI using ncurses.

Installation
---

The source code for Elma [is on Github](https://github.com/klavinslab/elma).
Also forked in this project.

From Dockerhub
---

To get started, you will need a C++ build environment. The Docker container `elma` is available on Dockerhub, which can be used as follows:

    git clone https://github.com/AnushaKamat/AdaptiveCruiseControl.git
    cd AdaptiveCruiseControl
    docker run -v $PWD:/source -it klavins/elma:latest bash
    make
    ./examples/bin/ACC



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
