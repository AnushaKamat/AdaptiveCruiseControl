#ifndef _RADAR_H
#define _RADAR_H

#include "elma/elma.h" // Note installation directory for elma


namespace driving_environment {

    using namespace std::chrono;
    using namespace elma;    

    class Radar : public Process {
        public:

        //! Wrap the base process class
        //! \param name The name of the car    
        Radar(std::string name) : Process(name) {}

        //Car(std::string name,car_type model) : Process(name) {}
            
        //! Nothing to do to initialize
        void init();

        //! To start a new simulation, this process sets
        //! the car's velocity to zero kph.    
        void start() {}

        //! The update method gets the latest force from the 
        //! Throttle Channel, if any. Then it updates the 
        //! car's velocity, and sends it out on the Velocity
        //! Channel.     
        void update();

        //! Nothing to do to stop    
        void stop() {}

        void set_obstacle_dist(int OD){
            obs_dist = OD;
        }
        int get_obstacle_dist(void){
            return obs_dist;
        }
        private:
        int obs_dist = 100;
        int ACC_status = 0; 
        
    }; 


}


#endif