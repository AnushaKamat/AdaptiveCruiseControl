#include <iostream>
#include <chrono>
#include "elma/elma.h" // Note installation directory for elma
#include "ACC.h"

//! \file

using namespace std::chrono;
using std::vector;
using namespace elma;
using namespace driving_environment;

//! Watch for events that change the status of ACC switch and if obstacle is found.
void AutoCruiseControl::init() {
    watch("ACC status", [this](Event& e) {
        ACC_status = e.value();
    });
    watch("Obstacle",[this](Event& e){
        obs_dist = e.value();
    });
}

//! Get the safe distance from SafetyDistance channel,
//! Get the desired speed from the DesSpeed channel,
//! Get the velocity from the Velocity Channel, compute
//! a simple proportional control law, and send the result
//! to the Throttle channel.  
//! If an obstacle is found within safe distance and  if velocity of car is above low speed
//! Decrease desired speed to avoid collision with obstacle    
void AutoCruiseControl::update() {
    if(ACC_status){
        if(channel("SafetyDistance").nonempty()){
            sfdist = channel("SafetyDistance").latest();
        }
        
        if ( channel("Velocity").nonempty() ) {
                speed = channel("Velocity").latest();
        }
        //std::cout << "obs_dist : "<<obs_dist <<" sfdist : "<<sfdist <<"\n";
        if(channel("DesSpeed").nonempty()){
                desired_speed = channel("DesSpeed").latest();
                
        }
        if(obs_dist < sfdist && speed >= lowspeed){                 
            desired_speed -=KpAd*(sfdist - obs_dist);
        }
        channel("Throttle").send(-KP*(speed - desired_speed)); //slow breaking is also required, and must be above idle
        
    }
}