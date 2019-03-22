#include <iostream>
#include <chrono>
#include "elma/elma.h" // Note installation directory for elma
#include "Car.h"

//! \file

using namespace std::chrono;
using std::vector;
using namespace elma;
using namespace driving_environment;


        //! To start a new simulation, this process sets
        //! the car's velocity to zero kph.    
        void Car::start() {
            velocity = 0;
        }

        //! The update method gets the latest force from the 
        //! Throttle Channel, if any. Then it updates the 
        //! car's velocity, and sends it out on the Velocity
        //! Channel.     
        void Car::update() {
           // std::cout <<"Actual Force :" << force << "\n";
           if ( channel("Throttle").nonempty() ) {
                force = channel("Throttle").latest();
            }
            velocity += ( delta() / 1000 ) * ( - k * velocity + force ) / m;
            channel("Velocity").send(velocity);
            std::cout << "milli_time(): " <<milli_time() <<" velocity : " << velocity << " \n";
            
        }