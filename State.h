#ifndef STATE_H
#define STATE_H

#include <array>

//[px, py, pz, vx, vy, vz, mass] rocker state vector
using State = std::array<float, 7>;

inline State operator+(const State& state, const State& b) {
    State result;
    for (size_t i = 0; i < state.size(); i++) result[i] = state[i] + b[i];
    return result;
}

inline State operator*(const State& state, float s) {
    State result;
    for (size_t i = 0; i < state.size(); i++) result[i] = state[i] * s;
    return result;
}

#endif