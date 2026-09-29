// Rigid-body 6-DOF equations of motion (Beard & McLain ch. 3, quaternion form
// from Appendix B). Forces and moments come in from outside, so this file knows
// nothing about aerodynamics, propulsion or gravity.
#pragma once

#include "bwb/math.hpp"
#include "bwb/params.hpp"

namespace bwb {

// 13-element state, B&M 2nd-ed ordering:
//   p_ned  position of the CG in NED, m               (pn, pe, pd)
//   v_body velocity of the CG in body axes, m/s       (u, v, w)
//   att    attitude quaternion                        (e0, e1, e2, e3)
//   w_body angular rate of body wrt NED, body axes    (p, q, r) rad/s
struct State {
    Vec3 p_ned;
    Vec3 v_body;
    Quat att;
    Vec3 w_body;
};

// Total external force and moment about the CG, both in body axes.
struct Wrench {
    Vec3 force;   // N    (fx, fy, fz)
    Vec3 moment;  // N m  (l, m, n)
};

// The Gamma coefficients of B&M ch. 3. They fold the inverse of the
// inertia matrix into the rotational equations; precompute once per aircraft.
struct InertiaGammas {
    double g1, g2, g3, g4, g5, g6, g7, g8;
    double Jy;
};

InertiaGammas make_gammas(const MassProps& mp);

// Time derivative of the state, x_dot = f(x, wrench).
// The derivative has the same layout as State (quaternion rates in `att`).
State derivatives(const State& x, const Wrench& fm, const MassProps& mp, const InertiaGammas& g);

// Gravity force in body axes: R_i^b * [0, 0, m g]  (B&M ch. 4, quaternion form).
Vec3 gravity_body(const Quat& att, double mass, double g0 = 9.80665);

// Angular momentum of the body, expressed in NED. Conserved when the moment is zero.
Vec3 angular_momentum_ned(const State& x, const MassProps& mp);

// Rotational kinetic energy 0.5 w^T J w. Conserved when the moment is zero.
double rotational_ke(const State& x, const MassProps& mp);

}  // namespace bwb
