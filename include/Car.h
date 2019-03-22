#ifndef _CAR_H
#define _CAR_H

#include "elma/elma.h" // Note installation directory for elma


namespace driving_environment {

    using namespace std::chrono;
    using namespace elma;    

    typedef enum { REGULAR, CC , ACC} car_type;
    const double KP = 314.15;

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
        void start();

        //! The update method gets the latest force from the 
        //! Throttle Channel, if any. Then it updates the 
        //! car's velocity, and sends it out on the Velocity
        //! Channel.     
        void update();

        //! Nothing to do to stop    
        void stop() {}

        double get_velocity(void){
            return velocity;
        }

        private:
        double velocity;
        double force = 0;
        const double k = 0.02;
        const double m = 1000;
        int status; // 0 - regular mode, 1 - CC 
        
    };  

}
    

#endif