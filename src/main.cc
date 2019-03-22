#include <iostream>
#include <chrono>
#include <vector>
#include <ncurses.h>

#include "Driver.h"

using namespace std::chrono;
using namespace elma;
using namespace driving_environment;

int main() {

    Manager m;

    Car acar("CarA"); // can specify type of car in constructor, Regular,CC,ACC,Camera
    CruiseControl cc("Control");
    Driver driver("Steve");
    AutoCruiseControl acc("AutoControl");
    Radar sens("RadarSensor");
    Channel throttle("Throttle");
    Channel velocity("Velocity");
    Channel des_speed("DesSpeed");
    Channel safetydistance("SafetyDistance");

    m.schedule(acar, 100_ms)
    .schedule(cc, 100_ms)
    .schedule(acc,100_ms)
    
    .schedule(driver, 5_s)
    .schedule(sens,1_s)
    .add_channel(throttle)
    .add_channel(velocity)
    .add_channel(des_speed)
    .add_channel(safetydistance)
    .init()
    //.simrun(40_s);
    //.run(40_s);
    .use_simulated_time()
    .run(60_s);
}