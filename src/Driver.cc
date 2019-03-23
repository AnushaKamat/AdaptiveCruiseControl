#include <iostream>
#include <chrono>
#include "elma/elma.h" // Note installation directory for elma
#include "Driver.h"

//! \file

using namespace std::chrono;
using std::vector;
using namespace elma;
using namespace driving_environment;


//! This is like a front end method.
//! User can set the parameters using setters
//! The update method depending upon the setting of ACC_on and CC_on 
//! switches , sets the operation mode for the car i.e; either REGULAR, CC or ACC
//! when both CC_on and ACC_on is OFF(0), then car operates in REGULAR mode
//! when ACC_on is set , car operates in ACC (Adaptive Cruise Control) mode
//! when CC_on is set, car operates in CC(Cruise Control) Mode
//! if both CC_on and ACC_on is set then ACC gets precedence and CC is internally is set to OFF
//! Depending on the operation mode set by driver, the events are emited.
//! if ACC, ACC status is emited, safety distance and Desired speed set by driver is
//! also sent on the channels SafetyDistance and DesSpeed respectively
//! In case of CC, CC status is emitted and desired_speed set by the driver is sent on 
//! DesDpeed channel
//! In case of Regular mode, the driver presses acceleration pedal (accped)
//! and the force applied is sent over Throttle channel
void Driver::update() {
    //std::cout<<"CC status : " <<CC_on << std::endl;
    //std::cout<<"ACC status : " <<ACC_on << std::endl;
    
    if(CC_on == 0 && ACC_on == 0 ){
        operation_mode = REGULAR;
        emit(Event("CC status",CC_on));
        emit(Event("ACC status",ACC_on));  
        channel("Throttle").send(KP*accped); 
    }
    else if(ACC_on == 1){
        operation_mode = ACC;
        if(CC_on){
            CC_on =0;
        }
        //std::cout<<"Desired Speed : " << desired_speed <<std::endl;
        //std::cout<<"Safety Distance : " << sfdist <<std::endl;
        emit(Event("ACC status",ACC_on));
        channel("SafetyDistance").send(sfdist);
        channel("DesSpeed").send(desired_speed);
    }
    else{
        operation_mode = CC;
        //std::cout<<"Desired Speed : " << desired_speed <<std::endl;
        emit(Event("CC status",CC_on));
        channel("DesSpeed").send(desired_speed);
    }  
}