#ifndef FUELTANK_H
#define FUELTANK_H

class FuelTank{
private:
    const float CAPACITY = 3800000;
    float fuel;

public:
    FuelTank() : fuel(CAPACITY){}
    
    FuelTank(float fuel_) : fuel(fuel_){}

    float GetFuel() const { return fuel; }

    bool IsEmpty() const { return fuel <= 0.0f; }

    float Consume(float amount){
        if (fuel <= 0) return 0;
        if (amount >= fuel) {
            float consumed = fuel;
            fuel = 0;
            return consumed;
        }
        fuel -= amount;
        return amount;
    }

    void SetFuel(float f) { 
        fuel = f < 0 ? 0 : f; 
    }
};
#endif