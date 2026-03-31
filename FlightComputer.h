#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include "SensorUnit.h"
#include "LaunchSequence.h"
#include "Vec3.h"
#include <chrono>
#include <thread>
#include <iostream>

class FlightComputer {
public:
    void run() {
        auto cycleTime = std::chrono::milliseconds(100);
        auto startAllCycles = std::chrono::steady_clock::now();
        auto start = startAllCycles;
        SensorUnit sensors("SU-10-ALT");
        std::thread sensorThread(&SensorUnit::run, &sensors);
        LaunchSequence ls;
        ls.transition(ls.PRELAUNCH);
        ls.transition(ls.IGNITION);
        ls.transition(ls.LIFTOFF);

        for (int i = 0; i < 50; i++){
            float alt = sensors.getLatestVote();
            if(ls.getState() == "LIFTOFF"){
                if(alt >= 1200.0) { ls.transition(ls.MAX_Q);}
            }
            else if(ls.getState() == "MAX_Q"){
                if(alt >= 1500.0) { ls.transition(ls.MECO);}
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            if (elapsed > cycleTime){
                std::cout << "OVERRUN cycle " << i << ": " << elapsed.count() << "ms" << std::endl;
            }
            else {
                while (std::chrono::steady_clock::now() < start + std::chrono::milliseconds(100)) {
                    // spin burns CPU but guarantees precision
                }
                auto totalCycle = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
                std::cout << "Cycle " << i << ": State=" << ls.getState() <<  " Altitude: " << alt << "ft" << " work=" << elapsed.count() << "ms total=" << totalCycle.count() << "ms" << std::endl;
            }
            start = std::chrono::steady_clock::now();
        }
        std::cout << "All Cycles " << ": " << "ms total=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startAllCycles).count() << "ms" << std::endl;
        sensors.stopped = true;
        sensorThread.join();
    }
};

#endif