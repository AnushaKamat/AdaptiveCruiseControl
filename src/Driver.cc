#include <iostream>
#include <chrono>
#include "elma/elma.h" // Note installation directory for elma
#include "Driver.h"

//! \file

using namespace std::chrono;
using std::vector;
using namespace elma;
using namespace driving_environment;

//! Example: A simulated driver, who keeps cycling between 50 and 60 kph.  See examples/driving.cc.


//! initialize the desired speed
void Driver::init() {
    //desired_speed = 50;
    //sfdist = 5;
}

//! If the desired speed is 50, change to 60,
//! otherwise change to 50.
void Driver::update() {
    //std::cout<<"operation mode : " <<operation_mode <<std::endl;
    //std::cout<<"CC status : " <<CC_on << std::endl;
    //std::cout<<"ACC status : " <<ACC_on << std::endl;
    if(CC_on == 0 && ACC_on == 0 ){
        operation_mode = REGULAR;
        emit(Event("CC status",CC_on));
        emit(Event("ACC status",ACC_on));   
    }
    else if(ACC_on == 1){
        operation_mode = ACC;
        if(CC_on){
            CC_on =0;
        }
    }
    else{
        operation_mode = CC;
    }
    
    switch (operation_mode){
        case ACC :      ACC_on = 1;
                        //std::cout <<"In ACC now, ACC_on : "<< ACC_on <<"operation_mode : "<<operation_mode<<"\n";
                       
                        emit(Event("ACC status",ACC_on));
                        /*if(desired_speed == 50){
                            desired_speed = 30;             //Driver must emit CC_on event - then car operates with CC mode or else bypasses directly to accped mode
                        } 
                        else {                            //Driver emits CC_off or -ve throttle >> CC mode off
                            desired_speed = 50;
                        }*/
                        
                        //sfdist = 4;

                        //std::cout <<"1.Desired Speed :  " << desired_speed <<std::endl;
                        //std::cout <<"2.Safety Distance :  "<<sfdist << std::endl;
                        channel("SafetyDistance").send(sfdist);
                        channel("DesSpeed").send(desired_speed);
                        //std::cout <<"here 2 \n";
                        //operation_mode = REGULAR;
                        break;

        case CC :       CC_on = 1;
                        emit(Event("CC status",CC_on));
                        //std::cout <<"WHYYYYY In CC now, CC_on : "<< CC_on <<"operation_mode : "<<operation_mode<<"\n";
                        /*if(desired_speed == 50){
                            desired_speed = 60;             //Driver must emit CC_on event - then car operates with CC mode or else bypasses directly to accped mode
                        } 
                        else {                            //Driver emits CC_off or -ve throttle >> CC mode off
                            desired_speed = 50;
                        }*/
                        //std::cout <<"here 2 \n";
                        channel("DesSpeed").send(desired_speed);
                        //operation_mode = REGULAR;
                        break;

        case REGULAR:   //accped =45;
                        //std::cout << "In regular accped mode : "<<accped <<"\n";
                        /*
                        if(ACC_on ==1){
                            operation_mode = ACC;
                        }
                        else
                            if (CC_on == 1){
                            operation_mode = CC;
                        }*/
                        channel("Throttle").send(KP*accped);

                        
                        break;
    }

    
}

double Driver::get_desired_speed(void){
    return desired_speed;
}


