#ifndef _DRIVER_H
#define _DRIVER_H

#include "elma/elma.h" // Note installation directory for elma
#include "ACC.h"
#include "CC.h"

namespace driving_environment {

    using namespace std::chrono;
    using namespace elma;    

    //! A Process class to represent a Driver to the Car who decides how to run the Car
    class Driver : public Process {

        public: 

        //! Wrap the base process class
        //! \param name The name of the Driver       
        Driver(std::string name) : Process(name) {}

        //! Nothing to initialize.
        void init(){}

        //! Nothing to do to start
        void start() {}

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
        void update();
        
        //! Nothing to do to stop
        void stop() {}

        //! Getters
        //! \param desired_speed is returned
        double get_desired_speed(void){return desired_speed;}

        //! \param ACC_on is returned
        bool get_ACC_status(void){
            return ACC_on;
        }
        //! \param CC_on is returned
        bool get_CC_status(void){
            return CC_on;
        }
        //! \param operation_mode which is type of car is returned
        car_type get_car_type(void){
            return operation_mode;
        }
        //! \param sfdist - safe distance set by driver is returned
        int get_safe_distance(void){
            return sfdist;
        }
        //! \param accped - the value of acceleration pedal pressed is returned
        double get_accped(void){
            return accped;
        }
        //setters

        //! \param ACC_on is set 
        void set_ACC_on(bool status){
            ACC_on = status;
        }
        //! \param CC_on is set 
        void set_CC_on(bool status){
            CC_on = status;
        }
        //! \param operation_mode is set 
        void set_car_type(car_type status){
            operation_mode = status;
        }
        //! \param accped is set 
        void set_accped(double accleration){
            accped = accleration;
        }
        //! \param desired_speed is set 
        void set_desired_speed(double DS){
            desired_speed =DS;
        }
        //! \param sfdist is set 
        void set_safe_distance(int SD){
            sfdist = SD;
        }

        private:
        double desired_speed =50;
        double accped =45;
        // For CC :
        // CC_on =1 and operation_mode = CC
        // For ACC :
        // ACC_on =1 and operation_mode = ACC
        bool CC_on = 0;
        bool ACC_on = 0;
        car_type operation_mode = REGULAR;
        int sfdist = 5;
    }; 

}

#endif