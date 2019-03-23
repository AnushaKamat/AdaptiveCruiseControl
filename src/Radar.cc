#include <iostream>
#include <chrono>
#include "elma/elma.h" // Note installation directory for elma
#include "Radar.h"

//! \file

using namespace std::chrono;
using std::vector;
using namespace elma;
using namespace driving_environment;

//Car(std::string name,car_type model) : Process(name) {}
    
//! Nothing to do to initialize
void Radar::init() {
    watch("ACC status", [this](Event& e) {
                ACC_status = e.value();
    });
}

//! The update method checks if ACC is on , 
//! It emits event Obstacle 
//! It gives out the distance of the obstacle from car that it can sense     
void Radar::update() {
    if(ACC_status){
    emit(Event("Obstacle",get_obstacle_dist()));
    //std::cout << " Obstacle distance set to : " << get_obstacle_dist() <<"\n";
    }
}