#ifndef WORLD_H
#define WORLD_H

#include "State.h"
#include "Vec3.h"
#include <cmath>

class World {
private:
    Vec3 gravity;

public:
    World() : gravity(0, 0, -9.8f) {}

    float getAirDensity(float altitude) {
        if (altitude < 0) altitude = 0;
        return 1.225f * exp(-altitude / 8500.0f);
    }
    
    Vec3 ComputeDrag(const State& state, float Cd, float A) {
        Vec3 pos(state[0], state[1], state[2]);
        Vec3 vel(state[3], state[4], state[5]);
        float airDensity = getAirDensity(pos.getZ());
        float speed = vel.Magnitude();
        if (speed < 0.001f) return Vec3(0, 0, 0);
        float dragMag = 0.5f * airDensity * speed * speed * Cd * A;
        return vel.Normalize() * (-dragMag);
    }

    Vec3 ComputeGravity(const State& state) {
        float mass = state[6];
        return gravity * mass;
    }
};

#endif