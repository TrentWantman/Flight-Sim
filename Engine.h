#ifndef ENGINE_H
#define ENGINE_H

#include "Bus.h"
#include "FuelTank.h"

class Engine
{
private:
    const double maxMassFlow = 21450.0;
    const double isp = 330.0;
    const double g0 = 9.80665;
    const double maxThrust = maxMassFlow * isp * g0;
    double throttle;
    Bus& bus;
    FuelTank& fuelTank;

public:

    Engine(Bus& bus_, FuelTank& fuelTank_): bus(bus_), fuelTank(fuelTank_), throttle(0.0){}
    Engine(Bus& bus_, FuelTank& fuelTank_, double throttle_): bus(bus_), fuelTank(fuelTank_), throttle(throttle_){}


    void Update(double dt) {
        ReadCommands();
        double fuelNeeded = maxMassFlow * throttle * dt;
        double fuelConsumed = fuelTank.Consume(fuelNeeded);
        if (fuelConsumed < fuelNeeded) {
            throttle = 0.0;
        }
    }

    void ReadCommands() {
        float cmd;
        if (bus.throttleChannel.read(cmd)) {
            SetThrottle(static_cast<double>(cmd));
        }
    }

    void SetThrottle(double t){
        if (t >= 1.0){
            throttle = 1.0;
        }
        else if(t <= 0.0){
            throttle = 0.0;
        }
        else{
            throttle = t;
        }
    }

    void SetThrust(double thrust) {
        if (thrust >= maxThrust){
            throttle = 1.0;
        }
        else if(thrust <= 0.0){
            throttle = 0.0;
        }
        else{
            throttle = thrust / maxThrust;
        }
    }

    double GetThrottle() const { return throttle; }

    double GetThrust() const { return maxMassFlow * throttle * isp * g0; }

    double GetBurnRate() const { return maxMassFlow * throttle; }

};

#endif