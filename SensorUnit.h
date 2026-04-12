#ifndef SENSORUNIT_H
#define SENSORUNIT_H

#include "Rocket.h"
#include "DoubleCircularBuffer.h"
#include <thread>
#include <iostream>
#include <string>
#include <mutex>
#include <cstdlib>

class SensorUnit {
private:
    const Rocket& rocket;
    float dt = 0.1f;
    Bus& bus;

public:
    std::string name;
    int sensortype;

    float s1_, s2_, s3_;
    bool s1_faulted = false;
    bool s2_faulted = false;
    bool s3_faulted = false;
    bool faulted = false;
    float latestVote_;

    bool stopped = false;

    std::mutex mtx_;

    SensorUnit(const std::string& name_, Bus& bus_, const Rocket& rocket_, int sensortype_) : name(name_), bus(bus),  faulted(false), latestVote_(0.0), rocket(rocket_), sensortype(sensortype_) {}

    float noise() {
        return ((rand() % 100) - 50) * 0.01;
    }

    float update(float s1, float s2, float s3){
        s1_ = s1; s2_ = s2; s3_ = s3;
        return vote();
    }

    float vote(){
        float vote;
        bool oneTwo = abs(s1_ - s2_) <= 0.5;
        bool oneThree = abs(s1_ - s3_) <= 0.5;
        bool twoThree = abs(s2_ - s3_) <= 0.5;
        int totalAgree = (int)oneTwo + (int)oneThree + (int)twoThree;
        if ( totalAgree > 1) {
            vote = (s1_ + s2_ + s3_) / 3;
        }
        else if (totalAgree > 0){
            if (oneTwo) {vote = (s1_ + s2_) / 2; s3_faulted = true;}
            else if (oneThree) {vote = (s1_ + s3_) / 2; s2_faulted = true;}
            else if (twoThree) {vote = (s2_ + s3_) / 2; s1_faulted = true;}
        }
        else{
            faulted = true;
            std::cout << "WARNING: Sensor Unit Is Faulty: sensor 1: " << s1_ << " sensor 2: " << s2_ << " sensor 3: " << s3_ << std::endl;
            return -999999.9;
        }
        return vote;
    }

    void run() {
        int cycle = 0;
        while (!stopped) {
            float value;
            if (sensortype == 0) value = rocket.GetPosition().getZ();
            else if (sensortype == 1) value = rocket.GetVelocity().getZ();
            else if (sensortype == 2) value = rocket.GetMass();
            
        {
            std::lock_guard<std::mutex> lock(mtx_);
            latestVote_ = update(value + noise(), value + noise(), value + noise());
            if (sensortype == 0) {
                bus.altitudeChannel.write(latestVote_);
                bus.altitudeChannel.swapBuffers();
            }
            else if (sensortype == 1) {
                bus.velocityChannel.write(latestVote_);
                bus.velocityChannel.swapBuffers();
            }
            else if (sensortype == 2) {
                bus.massChannel.write(latestVote_);
                bus.massChannel.swapBuffers();
            }
        }
            cycle++;
            std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(50));
        }
    }

    float getLatestVote() {
        std::lock_guard<std::mutex> lock(mtx_);
        return latestVote_;
    }
};

#endif