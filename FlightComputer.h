#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include "SensorUnit.h"
#include "LaunchSequence.h"
#include "Vec3.h"
#include "DoubleCircularBuffer.h"
#include <chrono>
#include <thread>
#include <iostream>

class FlightComputer {
private:

static constexpr float dt = 0.1f;


public:
    void run() {
        auto cycleTime = std::chrono::milliseconds(100);
        auto startAllCycles = std::chrono::steady_clock::now();
        auto start = startAllCycles;
        DoubleCircularBuffer altBuffer;
        DoubleCircularBuffer velBuffer;
        PhysicsModel model;
        SensorUnit altitudeSensors("SU-10-ALT", altBuffer, model, 0);
        SensorUnit velocitySensors("SU-10-VEL", velBuffer, model, 1);


        std::thread altitudeSensorThread(&SensorUnit::run, &altitudeSensors);
        std::thread velocitySensorThread(&SensorUnit::run, &velocitySensors);
        
        LaunchSequence ls;
        ls.transition(ls.PRELAUNCH);
        ls.transition(ls.IGNITION);
        ls.transition(ls.LIFTOFF);

        model.SetThrust(Vec3(0,0,15000.0f));

        int cycle = 0;

        while (ls.getState() != "SAFED"){
            cycle++;

            float alt = altitudeSensors.getLatestVote();
            float velocity = velocitySensors.getLatestVote();

            if(ls.getState() == "LIFTOFF"){ if(alt >= 1200.0) { ls.transition(ls.MAX_Q); model.SetThrust(Vec3(0,0,300.0f));}}
            else if(ls.getState() == "MAX_Q"){ if(alt >= 1500.0) { ls.transition(ls.MECO); model.SetThrust(Vec3(0,0,0.0f));} }
            else if(ls.getState() == "MECO"){ if(alt <= 1000.0 && velocity < 0) { ls.transition(ls.LANDING); model.SetThrust(Vec3(0,0,20000.0f));}}
            else if(ls.getState() == "LANDING"){ 
                if(alt <= 0.0) { ls.transition(ls.SAFED); model.SetThrust(Vec3(0,0,0)); }
                else if(velocity < -5.0f) { model.SetThrust(Vec3(0,0,20000.0f)); }
                else { model.SetThrust(Vec3(0,0,9800.0f)); }
            }

            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            if (elapsed > cycleTime){
                std::cout << "OVERRUN cycle " << cycle << ": " << elapsed.count() << "ms" << std::endl;
            }
            else {
                while (std::chrono::steady_clock::now() < start + std::chrono::milliseconds(100)) {
                    // spin burns CPU but guarantees precision
                }
                auto totalCycle = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
                std::cout << "Cycle " << cycle << ": State=" << ls.getState() <<  " Altitude: " << alt << "ft" << " Velocity: " << velocity << "ft/sec" << " work=" << elapsed.count() << "ms total=" << totalCycle.count() << "ms" << std::endl;
            }

            start = std::chrono::steady_clock::now();
        }
        std::cout << "All Cycles " << ": " << "ms total=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startAllCycles).count() << "ms" << std::endl;
        altitudeSensors.stopped = true;
        velocitySensors.stopped = true;
        altitudeSensorThread.join();
        velocitySensorThread.join();
    }

    FlightComputer(){}
};

#endif