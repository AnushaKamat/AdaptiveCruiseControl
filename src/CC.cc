#include <iostream>
#include <chrono>
#include "elma/elma.h" // Note installation directory for elma
#include "CC.h"

//! \file

using namespace std::chrono;
using std::vector;
using namespace elma;
using namespace driving_environment;

//! Example: A cruise controller for a Car process.  See examples/driving.cc.



//! Watch for events that change the desired speed.
void CruiseControl::init() {
    watch("CC status", [this](Event& e) {
        CC_status = e.value();
    });
}


//! Get the velocity from the Velocity Channel, compute
//! a simple proportional control law, and send the result
//! to the Throttle channel.    
void CruiseControl::update() {
    //std::cout << "Fom CC : "<<CC_status <<"\n";
    if(CC_status){
        if ( channel("Velocity").nonempty() ) {
            speed = channel("Velocity").latest();
        }
        if(channel("DesSpeed").nonempty()){
            desired_speed = channel("DesSpeed").latest();
        }
        //std::cout <<"Am I here ?? WHy \n";
        channel("Throttle").send(-KP*(speed - desired_speed));
    }
}


