#ifndef ENGINE_H
#define ENGINE_H

#include "Bus.h"
#include "FuelTank.h"

class Engine
{
private:
    const float maxMassFlow = 21450.0f;
    const float isp = 330.0f;
    float g0 = 9.80665f;
    const float maxThrust = maxMassFlow * isp * g0;
    float throttle;
    Bus& bus;
    FuelTank& fuelTank;

public:

    Engine(Bus& bus_, FuelTank& fuelTank_ ): bus(bus_), fuelTank(fuelTank_), throttle(0.f){}
    Engine(Bus& bus_, FuelTank& fuelTank_, float throttle_): bus(bus_), fuelTank(fuelTank_), throttle(throttle_){}


    void Update(float dt) {
        ReadCommands();
        float fuelNeeded = maxMassFlow * throttle * dt;
        float fuelConsumed = fuelTank.Consume(fuelNeeded);
        if (fuelConsumed < fuelNeeded) {
            throttle = 0.0f;
        }
    }

    void ReadCommands() {
        float cmd;
        if (bus.throttleChannel.read(cmd)) {
            SetThrottle(cmd);
        }
    }

    void SetThrottle(float t){
        if (t >= 1.0f){
            throttle = 1.0;
        }
        else if(t <= 0.0f){
            throttle = 0;
        }
        else{
            throttle = t;
        }
    }

    void SetThrust(float thrust) { 
        if (thrust >= maxThrust){
            throttle = 1.0;
        }
        else if(thrust <= 0.0f){
            throttle = 0;
        }
        else{
            throttle = thrust / maxThrust;
        }
    }

    float GetThrottle() const { return throttle; }

    float GetThrust() const { return maxMassFlow * throttle * isp * g0; }

    float GetBurnRate() const { return maxMassFlow * throttle; }

};

#endif