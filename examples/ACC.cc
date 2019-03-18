#include <iostream>
#include <chrono>
#include "elma.h"

//! \file

using namespace std::chrono;
using std::vector;
using namespace elma;

namespace driving_example {

    //! Example: Another car simulation process. See examples/driving.cc.
    typedef enum { REGULAR, CC , ACC} car_type;
    const double KP = 314.15;
    //! See the file examples/driving.cc for usage.
    class Car : public Process {
        public:

        //! Wrap the base process class
        //! \param name The name of the car    
        Car(std::string name) : Process(name) {}

        //Car(std::string name,car_type model) : Process(name) {}
            
        //! Nothing to do to initialize
        void init() {}

        //! To start a new simulation, this process sets
        //! the car's velocity to zero kph.    
        void start() {
            velocity = 0;
        }

        //! The update method gets the latest force from the 
        //! Throttle Channel, if any. Then it updates the 
        //! car's velocity, and sends it out on the Velocity
        //! Channel.     
        void update() {
           if ( channel("Throttle").nonempty() ) {
                force = channel("Throttle").latest();
            }
            velocity += ( delta() / 1000 ) * ( - k * velocity + force ) / m;
            channel("Velocity").send(velocity);
            std::cout << milli_time() << "," << velocity << " \n";
            
        }

        //! Nothing to do to stop    
        void stop() {}

        private:
        double velocity;
        double force;
        const double k = 0.02;
        const double m = 1000;
        int status; // 0 - regular mode, 1 - CC 
        
    };  
    
    //! Example: A cruise controller for a Car process.  See examples/driving.cc.

    //! See the file examples/driving.cc for usage.
    class CruiseControl : public Process {

        public:

        //! Wrap the base process class
        //! \param name The name of the controller      
        CruiseControl(std::string name) : Process(name) {}

        //! Watch for events that change the desired speed.
        void init() {
            watch("CC status", [this](Event& e) {
                CC_status = e.value();
            });
        }

        //! Nothing to do to initialize    
        void start() {}

        //! Get the velocity from the Velocity Channel, compute
        //! a simple proportional control law, and send the result
        //! to the Throttle channel.    
        void update() {
            if(CC_status){
                if ( channel("Velocity").nonempty() ) {
                    speed = channel("Velocity").latest();
                }
                if(channel("DesSpeed").nonempty()){
                    desired_speed = channel("DesSpeed").latest();
                }
                channel("Throttle").send(-KP*(speed - desired_speed));
            }
        }

        //! Nothing to do to stop    
        void stop() {}

        private:
        double speed = 0;
        double desired_speed = 0.0;
        int CC_status;    
    };

    //! Example: A cruise controller for a Car process.  See examples/driving.cc.

    //! See the file examples/driving.cc for usage.
    class AutoCruiseControl : public Process {

        public:

        //! Wrap the base process class
        //! \param name The name of the controller      
        AutoCruiseControl(std::string name) : Process(name) {}

        //! Watch for events that change the desired speed.
        void init() {
            watch("ACC status", [this](Event& e) {
                ACC_status = e.value();
            });
            watch("Obstacle",[this](Event& e){
                obs_dist = e.value();
            });
        }

        //! Nothing to do to initialize    
        void start() {}

        //! Get the velocity from the Velocity Channel, compute
        //! a simple proportional control law, and send the result
        //! to the Throttle channel.    
        void update() {
            if(ACC_status){
                if(channel("SafetyDistance").nonempty()){
                    sfdist = channel("SafetyDistance").latest();
                }
                
                if ( channel("Velocity").nonempty() ) {
                        speed = channel("Velocity").latest();
                }
                
                if(obs_dist > sfdist){
                    
                    if(channel("DesSpeed").nonempty()){
                        desired_speed = channel("DesSpeed").latest();
                        
                    }
                    channel("Throttle").send(-KP*(speed - desired_speed));
                    
                }
                else{
                    if(obs_dist < sfdist){
                        desired_speed -=5;
                        channel("Throttle").send(-KP*(speed - desired_speed));
                        
                    }
                }
            }
        }

        //! Nothing to do to stop    
        void stop() {}

        private:
        double speed = 0;
        double desired_speed = 0.0;
        int ACC_status;  
        int obs_dist; 
        int sfdist; 
    };


    class Radar : public Process {
        public:

        //! Wrap the base process class
        //! \param name The name of the car    
        Radar(std::string name) : Process(name) {}

        //Car(std::string name,car_type model) : Process(name) {}
            
        //! Nothing to do to initialize
        void init() {
            obs_dist = 10;
        }

        //! To start a new simulation, this process sets
        //! the car's velocity to zero kph.    
        void start() {}

        //! The update method gets the latest force from the 
        //! Throttle Channel, if any. Then it updates the 
        //! car's velocity, and sends it out on the Velocity
        //! Channel.     
        void update() {
            if(obs_dist == 10){
                obs_dist = 3;
            }
            else {                           
                obs_dist = 10;
            }
            emit(Event("Obstacle",obs_dist));
            //std::cout << "3. Obstacle distance set to : " << obs_dist <<"\n";
            
        }

        //! Nothing to do to stop    
        void stop() {}

        private:
        int obs_dist;
        
    };  
 


    //! Example: A simulated driver, who keeps cycling between 50 and 60 kph.  See examples/driving.cc.
    class Driver : public Process {

        public: 

        //! Wrap the base process class
        //! \param name The name of the controller       
        Driver(std::string name) : Process(name) {}

        //! initialize the desired speed
        void init() {
            desired_speed = 50;
            sfdist = 5;
        }

        //! Nothing to do to start
        void start() {}

        //! If the desired speed is 50, change to 60,
        //! otherwise change to 50.
        void update() {
            std::cout<<"operation mode : " <<operation_mode <<std::endl;
            if(CC_on == 0 && ACC_on == 0){
                operation_mode = REGULAR;
                
            }
            
            switch (operation_mode){
                case ACC :      ACC_on = 1;
                                emit(Event("ACC status",ACC_on));
                                if(desired_speed == 50){
                                    desired_speed = 70;             //Driver must emit CC_on event - then car operates with CC mode or else bypasses directly to accped mode
                                } 
                                else {                            //Driver emits CC_off or -ve throttle >> CC mode off
                                    desired_speed = 50;
                                }
                                
                                sfdist = 4;

                                std::cout <<"1.Desired Speed :  " << desired_speed <<std::endl;
                                std::cout <<"2.Safety Distance :  "<<sfdist << std::endl;
                                channel("SafetyDistance").send(sfdist);
                                channel("DesSpeed").send(desired_speed);
                                operation_mode = REGULAR;
                                break;
                case CC :           CC_on = 1;
                                    emit(Event("CC status",CC_on));
                                    if(desired_speed == 50){
                                        desired_speed = 60;             //Driver must emit CC_on event - then car operates with CC mode or else bypasses directly to accped mode
                                    } 
                                    else {                            //Driver emits CC_off or -ve throttle >> CC mode off
                                        desired_speed = 50;
                                    }
                                    
                                    channel("DesSpeed").send(desired_speed);
                                    operation_mode = REGULAR;

                                    
                                    
                                break;

                case REGULAR:   
                                
                                accped =45;
                                std::cout << "In regular accped mode";
                                
                                if(ACC_on ==1){
                                    operation_mode = ACC;
                                }
                                else
                                 if (CC_on == 1){
                                    operation_mode = CC;
                                }
                                

                                channel("Throttle").send(KP*accped);
                                break;
            }
            
        }
        
        //! Nothing to do to stop
        void stop() {}

        private:
        double desired_speed;
        double accped;
        // For CC :
        // CC_on =1 and operation_mode = CC
        // For ACC :
        // ACC_on =1 and operation_mode = ACC
        bool CC_on = 0;
        bool ACC_on = 1;
        car_type operation_mode = ACC;
        int sfdist;
    };
}

int main() {

    Manager m;

    driving_example::Car acar("CarA"); // can specify type of car in constructor, Regular,CC,ACC,Camera
    driving_example::CruiseControl cc("Control");
    driving_example::Driver driver("Steve");
    driving_example::AutoCruiseControl acc("AutoControl");
    driving_example::Radar sens("RadarSensor");
    Channel throttle("Throttle");
    Channel velocity("Velocity");
    //Channel ccstatus("CC status");
    //Channel brake("Brake");
    Channel des_speed("DesSpeed");
    Channel safetydistance("SafetyDistance");

    m.schedule(acar, 100_ms)
    .schedule(cc, 100_ms)
    .schedule(acc,100_ms)
    .schedule(sens,100_ms)
    .schedule(driver, 5_s)
    .add_channel(throttle)
    .add_channel(velocity)
    .add_channel(des_speed)
    .add_channel(safetydistance)
    .init()
    .simrun(40_s);
}