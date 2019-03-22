#include <iostream>
#include <chrono>
#include "elma/elma.h" // Note installation directory for elma
#include "ACC.h"

//! \file

using namespace std::chrono;
using std::vector;
using namespace elma;
using namespace driving_environment;






        //! Watch for events that change the desired speed.
        void AutoCruiseControl::init() {
            watch("ACC status", [this](Event& e) {
                ACC_status = e.value();
            });
            watch("Obstacle",[this](Event& e){
                obs_dist = e.value();
            });
        }

        //! Get the velocity from the Velocity Channel, compute
        //! a simple proportional control law, and send the result
        //! to the Throttle channel.    
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
                    //std::cout << "Desired Speed when obstacle detected : "<<desired_speed<<"\n";
                }
                //std::cout <<"Am i in ACC?\n";
                channel("Throttle").send(-KP*(speed - desired_speed)); //slow breaking is also required, and must be above idle
                
            }
        }

