#ifndef FUELTANK_H
#define FUELTANK_H

class FuelTank{
private:
    const double CAPACITY = 3800000.0;
    double fuel;

public:
    FuelTank() : fuel(CAPACITY){}

    FuelTank(double fuel_) : fuel(fuel_){}

    double GetFuel() const { return fuel; }

    bool IsEmpty() const { return fuel <= 0.0; }

    double Consume(double amount){
        if (fuel <= 0) return 0;
        if (amount >= fuel) {
            double consumed = fuel;
            fuel = 0;
            return consumed;
        }
        fuel -= amount;
        return amount;
    }

    void SetFuel(double f) {
        fuel = f < 0 ? 0 : f;
    }
};
#endif