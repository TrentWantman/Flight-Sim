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

Vec3 position;
Vec3 velocity;
Vec3 thrust;
Vec3 gravity;



public:
    void run() {
        auto cycleTime = std::chrono::milliseconds(100);
        auto startAllCycles = std::chrono::steady_clock::now();
        auto start = startAllCycles;
        DoubleCircularBuffer buffer;
        SensorUnit sensors("SU-10-ALT", buffer);
        std::thread sensorThread(&SensorUnit::run, &sensors);
        LaunchSequence ls;
        ls.transition(ls.PRELAUNCH);
        ls.transition(ls.IGNITION);
        ls.transition(ls.LIFTOFF);

        int cycle = 0;

        while (ls.getState() != "SAFED"){
            cycle++;

            velocity = velocity + (thrust + gravity) * dt;
            position = position + velocity * dt;

            position.print();
            buffer.write(position.getZ());
            buffer.swapBuffers();

            float alt = sensors.getLatestVote();
            if(ls.getState() == "LIFTOFF"){ if(alt >= 1200.0) { ls.transition(ls.MAX_Q); thrust = Vec3(0,0,300.0f);}}
            else if(ls.getState() == "MAX_Q"){ if(alt >= 1500.0) { ls.transition(ls.MECO); thrust = Vec3(0,0,0.0f);} }
            else if(ls.getState() == "MECO"){ if(alt >= 4000.0) { ls.transition(ls.LANDING); thrust = Vec3(0,0,-200.0f);}}
            else if(ls.getState() == "LANDING"){ 
                if(alt <= 0.0) { ls.transition(ls.SAFED); thrust = Vec3(0,0,0); }
                else if(alt <= 1000.0) { thrust = Vec3(0,0,20.0f); }
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
                std::cout << "Cycle " << cycle << ": State=" << ls.getState() <<  " Altitude: " << alt << "ft" << " work=" << elapsed.count() << "ms total=" << totalCycle.count() << "ms" << std::endl;
            }

            start = std::chrono::steady_clock::now();
        }
        std::cout << "All Cycles " << ": " << "ms total=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startAllCycles).count() << "ms" << std::endl;
        sensors.stopped = true;
        sensorThread.join();
    }

    FlightComputer() : position(0,0,0), velocity(0,0,0), thrust(0,0,500.0f), gravity(0,0,-9.8f) {}
};

#endif