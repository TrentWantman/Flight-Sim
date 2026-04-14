#ifndef RK4INTEGRATOR_H
#define RK4INTEGRATOR_H

#include "Integrator.h"
#include "State.h"

class RK4Integrator : public Integrator{
public:
    State step (const State& state, DerivativeFn fn, float t, float dt) override {
        State k1 = fn(t, state);
        State k2 = fn(t + dt/2, state + k1 * (dt / 2));
        State k3 = fn(t + dt/2, state + k2 * (dt / 2));
        State k4 = fn(t + dt, state + k3 * dt);
        return state + ((k1 + (k2 * 2) + (k3 * 2) + k4) * (dt/6.0f));
    }
};

#endif