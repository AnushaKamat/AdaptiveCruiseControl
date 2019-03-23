#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <float.h>
#include "gtest/gtest.h"
#include "Driver.h"

namespace {
 
    using namespace elma;
    using namespace driving_environment;

    double tolerance = 1;
    //! Simulation classes
    //! A Radar Class to simulate an obstacle that cycles from 15 to 3 units of distance and keep moving further
    class Obstacle : public Radar{
        public:
            Obstacle (std::string name) : Radar(name) {}
            void update(){
                if(get_obstacle_dist() == 15){      
                    set_obstacle_dist(3);
                }
                else{                           
                    set_obstacle_dist(get_obstacle_dist()+1);
                }
                emit(Event("Obstacle",get_obstacle_dist()));
            }
    };

    //! A Driver class to simulate a driver who drives car in Cruise Control Mode and
    //! switches between setting the desired_speed from 50 to 60 units of speed
    class SimCCDriver : public Driver{
        public:
        SimCCDriver(std::string name) : Driver(name){}
            void update() {
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
                    //std::cout << "New Set speed : " << get_desired_speed() <<std::endl;
                    channel("DesSpeed").send(get_desired_speed());
                }
                else{
                    channel("Throttle").send(KP*get_accped());
                }
            }
    };

    //! A Driver Class that simulates driving a Adaptive Cruise Control
    //! Here the driver cycles between setting desired speed from 30 to 50 kph
    //! Also this driver has kept the safety distance set as default and not changing it manually
    class SimACCDriver : public Driver{
        public:
        SimACCDriver(std::string name) : Driver(name){}
            void update() {
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
                    //std::cout << "New Set speed : " << get_desired_speed() <<std::endl;
                    channel("SafetyDistance").send(get_safe_distance());
                    channel("DesSpeed").send(get_desired_speed());
                }
                else{
                    channel("Throttle").send(KP*get_accped());
                }
            }
    };

    //! A Driver Class that simulates driving a Adaptive Cruise Control
    //! Here the driver sets a desired speed
    //! And keeps changing the safety distance in the driving 
    class SimACCDriver2 : public Driver{
        public:
        SimACCDriver2(std::string name) : Driver(name){}
            void update() {
                set_desired_speed(30);
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
                    //std::cout <<"Safety Distance is changed to : " << get_safe_distance()<<std::endl;
                    channel("SafetyDistance").send(get_safe_distance());
                    channel("DesSpeed").send(get_desired_speed());
                }
                else{
                    channel("Throttle").send(KP*get_accped());
                }
            }
    };

    //! Tests
    //! 1. Check operation in regular Mode : 
    //! No status is set with respect to CC/ACC
   
    TEST(RegularMode,Basic) { 
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
   
    //! 2. Check operation of CC Mode: 
    //! The set Speed is not Set and CC runs on default 
        TEST(CCMode,Basic) { 
        Manager m;
        Car car("Rhaegal"); 
        CruiseControl cc("Control");
        Driver driver("JonSnow");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");

        driver.set_CC_on(1);

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
        EXPECT_NEAR(car.get_velocity(),50,tolerance);
    }  
    // 3. Check operation in ACC mode : 
    //! Desired speed not set , so running on default ACC desired speed
    //! Desired safety distance is not set , on default safety distance
    //! Simple case with no obstacles detected during the ride 
    //! obstacle distance very high by default if not set
    TEST(AutoCruise,Basic) { 
        Manager m;

        Car car("Drogon"); 
        Driver driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Radar sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

        driver.set_ACC_on(1);   //Sets ACC_on 
    
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
        EXPECT_NEAR(car.get_velocity(),50,tolerance);
    }
    
    //! 4. CC with desired speed set:
    //! Driver changes desired_speed from 40 to 60 after switching off
    //! Driver finally switches off CC and runs Car in Regular Mode
     TEST(CCMode,VariedSpeed) { 
        Manager m;
        Car car("Rhaegal"); 
        CruiseControl cc("Control");
        Driver driver("JonSnow");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");

        driver.set_CC_on(1);
        driver.set_desired_speed(40);

        m.schedule(car, 100_ms)
        .schedule(cc, 100_ms)
        .schedule(driver, 5_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .init()
        .use_simulated_time()
        .run(40_s);

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
        EXPECT_GT(car.get_velocity(),60); //Due to constant accped at 45.
        EXPECT_EQ(driver.get_car_type(),REGULAR);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),0);
    }   

    //! 5. ACC with desired speed set:
    //! Driver changes desired_speed from 40 to 60 after switching off
    //! Driver finally switches off ACC and runs Car in Regular Mode
     TEST(AutoCruise,VariedSpeed) { 
         Manager m;
        Car car("Drogon"); 
        Driver driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Radar sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

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
        .use_simulated_time()
        .run(40_s);

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

    //! 6. Car enabled with ACC and set to constant set speed of 40kph and safe distance of 4km.
    //! When obstacle is detected the Car reduces speed even if it is in ACC mode
    TEST(AutoCruise,ObstacleDetection) { 
        Manager m;
        Car car("Drogon"); 
        Driver driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Obstacle sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

        driver.set_ACC_on(1);   
        driver.set_desired_speed(40);
        driver.set_safe_distance(4); //any obstacle more than or equal to 4 is ok and car should not reduce speed
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
        .use_simulated_time()
        .run(38_s);
     
        if(driver.get_safe_distance()<=sens.get_obstacle_dist()){
            EXPECT_EQ(driver.get_desired_speed(),40);
            EXPECT_NEAR(car.get_velocity(),40,1); 
        }
        else{
            EXPECT_EQ(driver.get_desired_speed(),40);
            EXPECT_LT(car.get_velocity(),40); 
        }
        
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);      
    } 

    //! 7. Check CC with changing set speed 
    //! The Driver cycles between 50 to 60kph
    TEST(CCMode,SimulatedDriver) { 
        Manager m;
        Car car("Rhaegal"); 
        CruiseControl cc("Control");
        SimCCDriver driver("JonSnow");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");

        driver.set_CC_on(1);
        
        m.schedule(car, 100_ms)
        .schedule(cc, 100_ms)
        .schedule(driver, 8_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .init()
        .use_simulated_time();
        m.run(64_s);

        EXPECT_NEAR(car.get_velocity(),60,tolerance);
        EXPECT_EQ(driver.get_car_type(),CC);
        EXPECT_EQ(driver.get_ACC_status(),0);        
        EXPECT_EQ(driver.get_CC_status(),1);

    }   

    //!! 8. Check ACC with changing set speed , no obstacle, simulated driver cycling between 50 - 30 
    //! Radar's default obsracle_dtsance is 100 if not set, so no Obstacle in this case
    TEST(AutoCruise,SimulatedDriver) { 
        Manager m;
        Car car("Drogon"); 
        SimACCDriver driver("Khaleesi");
        AutoCruiseControl acc("AutoControl");
        Radar sens("RadarSensor");

        Channel throttle("Throttle");
        Channel velocity("Velocity");
        Channel des_speed("DesSpeed");
        Channel safetydistance("SafetyDistance");

        driver.set_ACC_on(1);   
        driver.set_safe_distance(4); //any obstacle farther than 4 is ok and Car should not reduce speed
        
        m.schedule(car, 100_ms)
        .schedule(acc,100_ms)
        .schedule(driver, 10_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time()
        .run(50_s);
        EXPECT_NEAR(car.get_velocity(),50,tolerance);
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);
        EXPECT_EQ(driver.get_safe_distance(),4);
        EXPECT_EQ(sens.get_obstacle_dist(),100);      
    } 
   
    //! 9. ACC operation with a Simulated Driver with desired_speed set to 30
    //! the Simulated driver cycles between changing safety distance between 4 and 6
    //! The obstacle is constant at 5.
    //! ACC slows down when obstacle is found within the safety distance set.
    //! when sfdist = 6, speed should increase towards 30
    //! otherwise it should decrease (but not less than lowspeed)
    TEST(AutoCruise,VaryingSafetyDistance) { 
        
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
        .schedule(driver, 10_s)
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
            EXPECT_NEAR(car.get_velocity(),30,tolerance); //set speed
        }
        else{
            EXPECT_NEAR(car.get_velocity(),10,tolerance); //low speed
        }
        

    }

    //! 10. Check ACC with combination of changing speed and set distance.
    //! The Simulated Driver sets ACC_on and cycles between 50 -30kph
    //! The safe distance is set to 4.
    //! Obstacle is simulated to move 15 units away from Car and reset to 3 units away.
    //! 
     TEST(AutoCruise,ACCFullTest) { 
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
        .schedule(driver, 10_s)
        .schedule(sens,1_s)
        .add_channel(throttle)
        .add_channel(velocity)
        .add_channel(des_speed)
        .add_channel(safetydistance)
        .init()
        .use_simulated_time()
        .run(60_s);
     
        if(driver.get_safe_distance()<=sens.get_obstacle_dist()){
            EXPECT_NEAR(car.get_velocity(),30,tolerance); //set speed
        }
        else{
            EXPECT_NEAR(car.get_velocity(),10,tolerance); //low speed
        }
        EXPECT_EQ(driver.get_car_type(),ACC);
        EXPECT_EQ(driver.get_ACC_status(),1);        
        EXPECT_EQ(driver.get_CC_status(),0);
        EXPECT_EQ(driver.get_safe_distance(),4);
        
    } 
    
    //! 11. Switching from Regular to CC
    //! Switching from CC to ACC
    //by changing CC_on and ACC_on
    TEST(Regular,ToCCtoACC) { 
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

    //! 12. Switching from Regular to ACC
    //! and ACC to CC
    //by changing CC_on and ACC_on
    TEST(Regular,ToACCtoCC) { 
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

}