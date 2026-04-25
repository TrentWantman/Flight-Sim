#ifndef EULERINTEGRATOR_H
#define EULERINTEGRATOR_H

#include "Integrator.h"

class EulerIntegrator : public Integrator {
public:
    State step(const State& current, DerivativeFn fn, double t, double dt) override {
        State derivative = fn(t, current);
        State next;
        for (size_t i = 0; i < current.size(); i++) {
            next[i] = current[i] + derivative[i] * dt;
        }
        return next;
    }
};

#endif