#ifndef _RADAR_H
#define _RADAR_H

#include "elma/elma.h" // Note installation directory for elma


namespace driving_environment {

    using namespace std::chrono;
    using namespace elma;    
    //! A Process class to represent a simulated sensing module to detect obstacle distance
    class Radar : public Process {
        public:

        //! Wrap the base process class
        //! \param name The name of the sensor    
        Radar(std::string name) : Process(name) {}
            
        //! Watch for events that change the status of ACC switch
        void init();

        //! Nothing to do to start   
        void start() {}

        //! The update method checks if ACC is on , 
        //! It emits event Obstacle 
        //! It gives out the distance of the obstacle from car that it can sense 
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