// Fixed-step RK4 for the rigid-body state.
//
// RK4 is the B&M choice (ch. 3 design project) and is 4th-order accurate: halve
// dt and the error drops ~16x. The tests check exactly that. After each step the
// quaternion is renormalized, because RK4 lets |e| drift slowly away from 1.
#pragma once

#include "bwb/rigid_body.hpp"

namespace bwb {

// x + k * s, element-wise over the whole state.
inline State axpy(const State& x, const State& k, double s) {
    return {x.p_ned + k.p_ned * s,
            x.v_body + k.v_body * s,
            {x.att.e0 + k.att.e0 * s, x.att.e1 + k.att.e1 * s,
             x.att.e2 + k.att.e2 * s, x.att.e3 + k.att.e3 * s},
            x.w_body + k.w_body * s};
}

// One RK4 step of length dt [s] starting at time t.
// wrench_fn(t, state) -> Wrench supplies forces and moments, so gravity, aero,
// thrust and wind plug in from outside without touching the integrator.
template <class WrenchFn>
State rk4_step(const State& x, double t, double dt, const MassProps& mp,
               const InertiaGammas& g, WrenchFn&& wrench_fn) {
    const State k1 = derivatives(x, wrench_fn(t, x), mp, g);
    const State x2 = axpy(x, k1, dt / 2);
    const State k2 = derivatives(x2, wrench_fn(t + dt / 2, x2), mp, g);
    const State x3 = axpy(x, k2, dt / 2);
    const State k3 = derivatives(x3, wrench_fn(t + dt / 2, x3), mp, g);
    const State x4 = axpy(x, k3, dt);
    const State k4 = derivatives(x4, wrench_fn(t + dt, x4), mp, g);

    State out = axpy(x, k1, dt / 6);
    out = axpy(out, k2, dt / 3);
    out = axpy(out, k3, dt / 3);
    out = axpy(out, k4, dt / 6);
    out.att = out.att.normalized();
    return out;
}

}  // namespace bwb
