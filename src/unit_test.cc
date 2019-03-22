#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <float.h>
//#include"matplotlibcpp.h"
#include "gtest/gtest.h"
#include "Driver.h"

namespace {
 
    using namespace elma;
    using namespace driving_environment;

    double tolerance = 0.02;
    //Simulation classes
    class Obstacle : public Radar{
        public:
            Obstacle (std::string name) : Radar(name) {}
            void update(){
                if(get_obstacle_dist() == 7){  
                    set_obstacle_dist(3);
                }
                else{                           
                    set_obstacle_dist(get_obstacle_dist()+1);
                }
                emit(Event("Obstacle",get_obstacle_dist()));
                //std::cout  << " Obstacle distance in new class : " << get_obstacle_dist() <<"\n";
            }
    };

    class SimCCDriver : public Driver{
        public:
        SimCCDriver(std::string name) : Driver(name){}
            void update() {
                //std::cout<<"operation mode : " <<get_car_type() <<std::endl;
                //std::cout<<"CC status : " <<get_CC_status() << std::endl;
                //std::cout<<"Should increase if 50, decrease if 60 ,Desired speed : "<<get_desired_speed()<<std::endl;
                if(get_CC_status()){
                    set_car_type(CC);
                }
                else{
                    set_car_type(REGULAR);
                }
                emit(Event("CC status",get_CC_status()));
                if(get_CC_status()){
                    if(get_desired_speed() == 50){
                        set_desired_speed(60);             //Driver must emit CC_on event - then car operates with CC mode or else bypasses directly to accped mode
                    } 
                    else {                            //Driver emits CC_off or -ve throttle >> CC mode off
                        set_desired_speed(50);
                    }
                    channel("DesSpeed").send(get_desired_speed());
                }
                else{
                    channel("Throttle").send(KP*get_accped());
                }
            }
    };

    class SimACCDriver : public Driver{
        public:
        SimACCDriver(std::string name) : Driver(name){}
            void update() {
                
                //std::cout<<"CC status : " <<get_CC_status() << std::endl;
                //std::cout<<"Should decrease if 50, increase if 30 ,Desired speed : "<<get_desired_speed()<<std::endl;
                //std::cout << "Safe Distance set : " <<get_safe_distance()<<std::endl;
                if(get_ACC_status()){
                    set_car_type(ACC);
                }
                else{
                    set_car_type(REGULAR);
                }

                emit(Event("ACC status",get_ACC_status()));
                if(get_ACC_status()){
                    if(get_desired_speed() == 50){
                        set_desired_speed(30);             //Driver must emit CC_on event - then car operates with CC mode or else bypasses directly to accped mode
                    } 
                    else {                            //Driver emits CC_off or -ve throttle >> CC mode off
                        set_desired_speed(50);
                    }
                    channel("SafetyDistance").send(get_safe_distance());
                    channel("DesSpeed").send(get_desired_speed());
                }
                else{
                    channel("Throttle").send(KP*get_accped());
                }
            }
    };

    class SimACCDriver2 : public Driver{
        public:
        SimACCDriver2(std::string name) : Driver(name){}
            void update() {
                set_desired_speed(50);
                //std::cout<<"operation mode : " <<get_car_type() <<std::endl;
                //std::cout<<"CC status : " <<get_CC_status() << std::endl;
                //std::cout<<"Speed is set to 50 : "<<get_desired_speed()<<std::endl;
                //std::cout << "If 4, it should be in 50, if 6 it should be less. Safe Distance set : " <<get_safe_distance()<<std::endl;
                if(get_ACC_status()){
                    set_car_type(ACC);
                }
                else{
                    set_car_type(REGULAR);
                }

                emit(Event("ACC status",get_ACC_status()));
                if(get_ACC_status()){
                    if(get_safe_distance() == 4){
                        set_safe_distance(6);             //Driver must emit CC_on event - then car operates with CC mode or else bypasses directly to accped mode
                    } 
                    else {                            //Driver emits CC_off or -ve throttle >> CC mode off
                        set_safe_distance(4);
                    }
                    channel("SafetyDistance").send(get_safe_distance());
                    channel("DesSpeed").send(get_desired_speed());
                }
                else{
                    channel("Throttle").send(KP*get_accped());
                }
            }
    };
    //Write Tests
    // 1. Check regular Mode
    
    TEST(AutoCruise,NormalMode) { 
        Manager m;

        Car car("Viserion"); 
        Driver driver("NightKing");
        Channel throttle("Throttle");
        Channel velocity("Velocity");

        m.schedule(car, 100_ms)        
        .schedule(driver, 5_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .init()
        .use_simulated_time()
        .run(30_s);
        EXPECT_EQ(driver.get_car_type(),REGULAR);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),0);
        
    }
    
    // 2. Check CC Mode with changing  speed
        TEST(AutoCruise,CCMode) { 
        Manager m;
        Car car("Rhaegal"); 
        CruiseControl cc("Control");
        Driver driver("JonSnow");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");

        driver.set_CC_on(1);
        //driver.set_car_type(CC);
        //Desired speed not set , so running on default CC desired speed

        m.schedule(car, 100_ms)
        .schedule(cc, 100_ms)
        .schedule(driver, 5_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .init()
        .use_simulated_time()
        .run(30_s);

        EXPECT_EQ(driver.get_car_type(),CC);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),1);
    }   
    // 3. Check ACC mode with changing  speed
        TEST(AutoCruise,Construction2) { 
        Manager m;

        Car car("Drogon"); 
        Driver driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Radar sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

        
        //driver.set_car_type(ACC); //Driver picks up car of his choice REGULAR,CC,ACC
        driver.set_ACC_on(1);   //Sets ACC_on 
        //Desired speed not set , so running on default CC desired speed
        //Desired safety distance is not set , on default safety distance
        //Simple case with no obstacles detected during the ride //obstacle distance very high by default if not set


        m.schedule(car, 100_ms)
        .schedule(acc,100_ms)
        .schedule(driver, 5_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time()
        .run(30_s);

        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);
    }
    

    //4. CC with desired speed varied, also checking switching off CC and switching to REGULAR mode
     TEST(AutoCruise,CCModeVariedSpeed) { 
        Manager m;
        Car car("Rhaegal"); 
        CruiseControl cc("Control");
        Driver driver("JonSnow");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");

        driver.set_CC_on(1);
        //driver.set_car_type(CC);

        //Desired speed not set , so running on default CC desired speed
        driver.set_desired_speed(40);
        m.schedule(car, 100_ms)
        .schedule(cc, 100_ms)
        .schedule(driver, 5_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .init()
        .use_simulated_time();
        EXPECT_EQ(driver.get_desired_speed(),40);
        m.run(40_s);
        EXPECT_EQ(driver.get_desired_speed(),40);
        EXPECT_NEAR(car.get_velocity(),40,tolerance);
        EXPECT_EQ(driver.get_car_type(),CC);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),1);

        driver.set_desired_speed(60);
        m.run(40_s);
        EXPECT_NEAR(car.get_velocity(),60,tolerance);
        EXPECT_EQ(driver.get_car_type(),CC);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),1);

        driver.set_CC_on(0);
        m.run(20_s);
        EXPECT_GT(car.get_velocity(),60);
        EXPECT_EQ(driver.get_car_type(),REGULAR);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),0);
    }   
    //5. ACC with desired speed varied , Also checking switching off ACC
     TEST(AutoCruise,ACCModeVariedSpeed) { 
         Manager m;
        Car car("Drogon"); 
        Driver driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Radar sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

        
        //driver.set_car_type(ACC); //Driver picks up car of his choice REGULAR,CC,ACC
        driver.set_ACC_on(1);   
        driver.set_desired_speed(40);

        m.schedule(car, 100_ms)
        .schedule(acc,100_ms)
        .schedule(driver, 5_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time();
        EXPECT_EQ(driver.get_desired_speed(),40);
        m.run(40_s);
        EXPECT_EQ(driver.get_desired_speed(),40);
        EXPECT_NEAR(car.get_velocity(),40,tolerance);
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);
     

        driver.set_desired_speed(60);
        m.run(40_s);
        EXPECT_NEAR(car.get_velocity(),60,tolerance);
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);

        driver.set_ACC_on(0);
        m.run(20_s);
        EXPECT_GT(car.get_velocity(),60);
        EXPECT_EQ(driver.get_car_type(),REGULAR);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),0);
    } 


    // 6. Check ACC with constant set speed and obstacle detection
     TEST(AutoCruise,ACCModeSetSpeedwithObstacle) { 
        
        Manager m;
        Car car("Drogon"); 
        Driver driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Obstacle sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

        
        //driver.set_car_type(ACC); //Driver picks up car of his choice REGULAR,CC,ACC
        driver.set_ACC_on(1);   
        driver.set_desired_speed(40);
        driver.set_safe_distance(4); //any obstacle more than 5 is ok and should not reduce speed
        sens.set_obstacle_dist(4);      //Obstacle is at safe distance and is moving

        m.schedule(car, 100_ms)
        .schedule(acc,100_ms)
        .schedule(driver, 5_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time();
        m.run(40_s);
        //std::cout<<"Safe Distance set by Driver : "<<driver.get_safe_distance()<<"\n";
        //std::cout<<"Obstacle at a distance of "<<sens.get_obstacle_dist()<<"from the car\n";
        if(driver.get_safe_distance()<=sens.get_obstacle_dist()){
            EXPECT_EQ(driver.get_desired_speed(),40);
            EXPECT_NEAR(car.get_velocity(),40,5); 
        }
        else{
            EXPECT_EQ(driver.get_desired_speed(),40);
            EXPECT_LT(car.get_velocity(),40); 
        }
        
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);
        

    } 


    // 7. Check CC with changing set speed 
    TEST(AutoCruise,CCModeAutoVariedSpeed) { 
        Manager m;
        Car car("Rhaegal"); 
        CruiseControl cc("Control");
        SimCCDriver driver("JonSnow");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");

        driver.set_CC_on(1);
        //driver.set_car_type(CC);

        //Desired speed is cycled between 50 to 60 by simulated driver class
        
        m.schedule(car, 100_ms)
        .schedule(cc, 100_ms)
        .schedule(driver, 5_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .init()
        .use_simulated_time();
        m.run(60_s);
        EXPECT_EQ(driver.get_car_type(),CC);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),1);

    }   

    //8. Check ACC with changing set speed , no obstacle, simulated driver cycling between 50 - 30 
    TEST(AutoCruise,ACCModeAutoSetSpeedwithNoObstacle) { 
        
        Manager m;
        Car car("Drogon"); 
        SimACCDriver driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Radar sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

        
        //driver.set_car_type(ACC); //Driver picks up car of his choice REGULAR,CC,ACC
        driver.set_ACC_on(1);   
        
        driver.set_safe_distance(4); //any obstacle more than 5 is ok and should not reduce speed
        

        m.schedule(car, 100_ms)
        .schedule(acc,100_ms)
        .schedule(driver, 5_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time();
        m.run(40_s);
        
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);
        EXPECT_EQ(driver.get_safe_distance(),4);
        EXPECT_EQ(sens.get_obstacle_dist(),100);
        

    } 
    
    //9. Check ACC with changing set distance
        TEST(AutoCruise,ACCModeAutoSetSpeedwithAutoSetDistance) { 
        
        Manager m;
        Car car("Drogon"); 
        SimACCDriver2 driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Radar sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

        
        //driver.set_car_type(ACC); //Driver picks up car of his choice REGULAR,CC,ACC
        driver.set_ACC_on(1);   
        sens.set_obstacle_dist(5);
        
        

        m.schedule(car, 100_ms)
        .schedule(acc,100_ms)
        .schedule(driver, 5_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time();
        m.run(50_s);
        
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);
        EXPECT_EQ(sens.get_obstacle_dist(),5);
        
        if(driver.get_safe_distance()<=sens.get_obstacle_dist()){
            EXPECT_NEAR(car.get_velocity(),40,2);
        }
        else{
            EXPECT_NEAR(car.get_velocity(),10,2);
        }
        

    }
    
    //10. Check ACC with combination of changing speed and set distance
     TEST(AutoCruise,ACCModewithVariedSpeedwithObstacle) { 
        Manager m;
        Car car("Drogon"); 
        SimACCDriver driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Obstacle sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

        
        //driver.set_car_type(ACC); //Driver picks up car of his choice REGULAR,CC,ACC
        driver.set_ACC_on(1);   
        
        driver.set_safe_distance(4); //any obstacle more than 5 is ok and should not reduce speed
        sens.set_obstacle_dist(4);      //Obstacle is at safe distance and is moving

        m.schedule(car, 100_ms)
        .schedule(acc,100_ms)
        .schedule(driver, 5_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time();
        m.run(40_s);
     
        
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);
        EXPECT_EQ(driver.get_safe_distance(),4);
        

    } 
    


    // 13. Switching from Regular to CC
    TEST(AutoCruise,NormalModetoCCtoACC) { 
        Manager m;

        Car car("Viserion"); 
        Driver driver("NightKing");
        AutoCruiseControl acc("AutoControl");
        Radar sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");
        
        m.schedule(car, 100_ms)
        .schedule(acc,100_ms)
        .schedule(driver, 5_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time()
        .run(10_s);

        EXPECT_EQ(driver.get_car_type(),REGULAR);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),0);
        
        driver.set_CC_on(1);
        m.run(10_s);
        
        EXPECT_EQ(driver.get_car_type(),CC);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),1);

        driver.set_ACC_on(1);
        m.run(10_s);
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);
        
    }

    // 14. Switching from Regular to ACC
    TEST(AutoCruise,NormalModetoACCtoCC) { 
        Manager m;

        Car car("Viserion"); 
        Driver driver("NightKing");
        AutoCruiseControl acc("AutoControl");
        Radar sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");
        
        m.schedule(car, 100_ms)
        .schedule(acc,100_ms)
        .schedule(driver, 5_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time()
        .run(10_s);

        EXPECT_EQ(driver.get_car_type(),REGULAR);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),0);

        driver.set_ACC_on(1);
        m.run(10_s);
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);

        driver.set_CC_on(1); // switching CC when ACC is on , ACC takes precedence
         m.run(10_s);
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);

        driver.set_ACC_on(0);
        driver.set_CC_on(1);
         m.run(10_s);
        EXPECT_EQ(driver.get_car_type(),CC);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),1);
        
    }

//17. EXceptions:
        //Edge Case what happens if he sets car_yype something else - cant select car type , only can set cc/acc
        // IF car is selected as ACC and CC is on , how should it reatc ?? - DONE
        // if any other car type is presssed  how to react - throws error 
        // if both ACC and CC is on 
        // if AA/CC with nothing on 
        //DONE



    
    //Clean up code 
    // 12. Doxygen
    // 13. Readme
    // 11. Block Diagram
    // Graph plotting
    // 14. Ncurses
    

    // More tests go here. You should aim to test every
    // method of every object, either directly or indirectly,
    // although testing user interfaces is notoriously 
    // difficult.

}