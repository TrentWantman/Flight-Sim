#ifndef WORLD_H
#define WORLD_H

#include "Rocket.h"
#include <cmath>

class World {
private:
    Vec3 gravity;

public:
    World() : gravity(0, 0, -9.8f) {}

    Vec3 ComputeForces(const Rocket& rocket) {
        Vec3 weight = gravity * rocket.GetMass();
        float airDensity = getAirDensity(rocket.GetPosition().getZ());
        Vec3 velocity = rocket.GetVelocity();
        float speed = velocity.Magnitude();
        float dragMagnitude = 0.5f * airDensity * speed * speed * rocket.GetDragCoef() * rocket.GetArea();
        Vec3 drag = velocity.Normalize() * (-dragMagnitude);
        return weight + drag;
    }

    float getAirDensity(float altitude) {
        if (altitude < 0) altitude = 0;
        return 1.225f * exp(-altitude / 8500.0f);
    }
};

#endif