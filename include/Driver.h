#ifndef _DRIVER_H
#define _DRIVER_H

#include "elma/elma.h" // Note installation directory for elma
#include "ACC.h"
#include "CC.h"

namespace driving_environment {

    using namespace std::chrono;
    using namespace elma;    


    

    class Driver : public Process {

        public: 

        //! Wrap the base process class
        //! \param name The name of the controller       
        Driver(std::string name) : Process(name) {}

        //! initialize the desired speed
        void init();

        //! Nothing to do to start
        void start() {}

        //! If the desired speed is 50, change to 60,
        //! otherwise change to 50.
        void update();
        
        
        //! Nothing to do to stop
        void stop() {}

        //Getters

        double get_desired_speed(void);
        bool get_ACC_status(void){
            return ACC_on;
        }

        bool get_CC_status(void){
            return CC_on;
        }

        car_type get_car_type(void){
            return operation_mode;
        }

        int get_safe_distance(void){
            return sfdist;
        }
        double get_accped(void){
            return accped;
        }
        //setters
        void set_ACC_on(bool status){
            ACC_on = status;
        }

        void set_CC_on(bool status){
            CC_on = status;
        }

        void set_car_type(car_type status){
            operation_mode = status;
        }
        void set_accped(double accleration){
            accped = accleration;
        }

        void set_desired_speed(double DS){
            desired_speed =DS;
        }

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