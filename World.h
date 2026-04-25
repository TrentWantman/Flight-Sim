#ifndef WORLD_H
#define WORLD_H

#include "State.h"
#include "Vec3.h"
#include <cmath>

class World {
public:
    static constexpr double EARTH_RADIUS = 6371000.0;
    static constexpr double MU = 3.986e14;

    double getAirDensity(double altitude) {
        if (altitude < 0) altitude = 0;
        return 1.225 * std::exp(-altitude / 8500.0);
    }

    Vec3 ComputeGravity(const State& state) {
        double mass = state[6];
        double px = state[0], py = state[1], pz = state[2];
        double r = std::sqrt(px*px + py*py + pz*pz);
        if (r < 1.0) r = 1.0;
        double factor = -MU / (r * r * r);
        double fx = (px * factor) * mass;
        double fy = (py * factor) * mass;
        double fz = (pz * factor) * mass;
        return Vec3(static_cast<float>(fx), static_cast<float>(fy), static_cast<float>(fz));
    }

    Vec3 ComputeDrag(const State& state, double Cd, double A) {
        double px = state[0], py = state[1], pz = state[2];
        double vx = state[3], vy = state[4], vz = state[5];
        double r = std::sqrt(px*px + py*py + pz*pz);
        double altitude = r - EARTH_RADIUS;
        if (altitude < 0) altitude = 0;
        double airDensity = getAirDensity(altitude);
        double speed = std::sqrt(vx*vx + vy*vy + vz*vz);
        if (speed < 0.001) return Vec3(0, 0, 0);
        double dragMag = 0.5 * airDensity * speed * speed * Cd * A;
        double invSpeed = -dragMag / speed;
        return Vec3(static_cast<float>(vx * invSpeed),
                    static_cast<float>(vy * invSpeed),
                    static_cast<float>(vz * invSpeed));
    }

    double GetAltitude(const State& state) {
        double px = state[0], py = state[1], pz = state[2];
        return std::sqrt(px*px + py*py + pz*pz) - EARTH_RADIUS;
    }
};

#endif