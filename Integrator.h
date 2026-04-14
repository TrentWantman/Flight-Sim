#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "State.h"
#include <functional>

using DerivativeFn = std::function<State(float t, const State& s)>;

class Integrator {
public:
    virtual ~Integrator() = default;
    virtual State step(const State& state, DerivativeFn fn, float t, float dt) = 0;
};

#endif