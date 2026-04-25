#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "State.h"
#include <functional>

using DerivativeFn = std::function<State(double t, const State& s)>;

class Integrator {
public:
    virtual ~Integrator() = default;
    virtual State step(const State& state, DerivativeFn fn, double t, double dt) = 0;
};

#endif