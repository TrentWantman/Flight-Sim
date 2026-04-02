#ifndef SENSORUNIT_H
#define SENSORUNIT_H

#include "PhysicsModel.h"
#include "DoubleCircularBuffer.h"
#include <thread>
#include <iostream>
#include <string>
#include <mutex>
#include <cstdlib>

class SensorUnit {
private:
    PhysicsModel& model;
    float dt = 0.1f;

public:
    std::string name;
    int sensortype;

    DoubleCircularBuffer& buffer;
    float s1_, s2_, s3_;
    bool s1_faulted = false;
    bool s2_faulted = false;
    bool s3_faulted = false;
    bool faulted = false;
    float latestVote_;

    bool stopped = false;

    std::mutex mtx_;

    SensorUnit(const std::string& name_, DoubleCircularBuffer& buffer_, PhysicsModel& model_, int sensortype_) : name(name_), buffer(buffer_),  faulted(false), latestVote_(0.0), model(model_), sensortype(sensortype_) {}

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
            model.Update(dt);
            float value;
            if(sensortype == 0){
                value = model.GetPosition().getZ();
            }
            else{
                value = model.GetVelocity().getZ();

            }
            
        {
            std::lock_guard<std::mutex> lock(mtx_);
            latestVote_ = update(value + noise(), value + noise(), value + noise());
            buffer.write(latestVote_);
            buffer.swapBuffers();
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