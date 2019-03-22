#ifndef _ACC_H
#define _ACC_H

#include "elma/elma.h" // Note installation directory for elma
#include "Car.h"
#include "Radar.h"

namespace driving_environment {

    using namespace std::chrono;
    using namespace elma;    


    class AutoCruiseControl : public Process {

        public:

        //! Wrap the base process class
        //! \param name The name of the controller      
        AutoCruiseControl(std::string name) : Process(name) {}

        //! Watch for events that change the desired speed.
        void init();

        //! Nothing to do to initialize    
        void start() {}

        //! Get the velocity from the Velocity Channel, compute
        //! a simple proportional control law, and send the result
        //! to the Throttle channel.    
        void update();

        //! Nothing to do to stop    
        void stop() {}

        private:
        double speed = 0;
        double desired_speed = 0.0;
        int ACC_status = 0;  
        int obs_dist = 10; 
        int sfdist = 4; //Create macro for sfdist, obsdist etc
        const double KpAd = 50;
        const double lowspeed =10;
    };




}

#endif