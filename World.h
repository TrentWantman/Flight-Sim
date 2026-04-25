#ifndef WORLD_H
#define WORLD_H

#include "State.h"
#include "Vec3.h"
#include <cmath>

class World {
public:
    static constexpr float EARTH_RADIUS = 6371000.0f;
    static constexpr float MU = 3.986e14f;

    float getAirDensity(float altitude) {
        if (altitude < 0) altitude = 0;
        return 1.225f * exp(-altitude / 8500.0f);
    }

    Vec3 ComputeGravity(const State& state) {
        float mass = state[6];
        Vec3 pos(state[0], state[1], state[2]);
        float r = pos.Magnitude();
        if (r < 1.0f) r = 1.0f;
        Vec3 gravAccel = pos * (-MU / (r * r * r));
        return gravAccel * mass;
    }

    Vec3 ComputeDrag(const State& state, float Cd, float A) {
        Vec3 pos(state[0], state[1], state[2]);
        Vec3 vel(state[3], state[4], state[5]);
        float altitude = pos.Magnitude() - EARTH_RADIUS;
        if (altitude < 0) altitude = 0;
        float airDensity = getAirDensity(altitude);
        float speed = vel.Magnitude();
        if (speed < 0.001f) return Vec3(0, 0, 0);
        float dragMag = 0.5f * airDensity * speed * speed * Cd * A;
        return vel.Normalize() * (-dragMag);
    }

    float GetAltitude(const State& state) {
        Vec3 pos(state[0], state[1], state[2]);
        return pos.Magnitude() - EARTH_RADIUS;
    }
};

#endif