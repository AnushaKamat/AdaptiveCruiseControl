#ifndef _CC_H
#define _CCC_H

#include "elma/elma.h" // Note installation directory for elma
#include "Car.h"


namespace driving_environment {

    using namespace std::chrono;
    using namespace elma;    
    
    //! A Process class to represent Car being run in Cruise Control mode
    class CruiseControl : public Process {

        public:

        //! Wrap the base process class
        //! \param name The name of the controller      
        CruiseControl(std::string name) : Process(name) {}

        //! Watch for events that change the status of CC switch.
        void init();

        //! Nothing to do to initialize    
        void start() {}

        //! Get the velocity from the Velocity Channel, 
        //! Get the desired speed from DesSpeed Channel,compute
        //! a simple proportional control law, and send the result
        //! to the Throttle channel.    
        void update();

        //! Nothing to do to stop    
        void stop() {}

        private:
        double speed = 0;
        double desired_speed = 0.0;
        int CC_status = 0;    
    };
}

#endif