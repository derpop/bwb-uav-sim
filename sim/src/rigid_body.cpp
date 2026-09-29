#include "bwb/rigid_body.hpp"

namespace bwb {

InertiaGammas make_gammas(const MassProps& mp) {
    const double Jx = mp.Jx, Jy = mp.Jy, Jz = mp.Jz, Jxz = mp.Jxz;
    const double G = Jx * Jz - Jxz * Jxz;  // B&M ch. 3
    return {Jxz * (Jx - Jy + Jz) / G,
            (Jz * (Jz - Jy) + Jxz * Jxz) / G,
            Jz / G,
            Jxz / G,
            (Jz - Jx) / Jy,
            Jxz / Jy,
            ((Jx - Jy) * Jx + Jxz * Jxz) / G,
            Jx / G,
            Jy};
}

State derivatives(const State& x, const Wrench& fm, const MassProps& mp, const InertiaGammas& g) {
    const double u = x.v_body.x, v = x.v_body.y, w = x.v_body.z;
    const double p = x.w_body.x, q = x.w_body.y, r = x.w_body.z;
    const Quat& e = x.att;
    const double l = fm.moment.x, m = fm.moment.y, n = fm.moment.z;

    State d;

    // Translational kinematics: position rate is body velocity rotated into NED.
    d.p_ned = rot_body_to_ned(e) * x.v_body;

    // Translational dynamics, B&M ch. 3: Newton's law written in the rotating
    // body frame picks up the -w x v transport term.
    d.v_body = Vec3{r * v - q * w, p * w - r * u, q * u - p * v} + fm.force / mp.mass;

    // Rotational kinematics, B&M App. B: e_dot = 0.5 * Omega(w) * e.
    d.att = {0.5 * (-p * e.e1 - q * e.e2 - r * e.e3),
             0.5 * (p * e.e0 + r * e.e2 - q * e.e3),
             0.5 * (q * e.e0 - r * e.e1 + p * e.e3),
             0.5 * (r * e.e0 + q * e.e1 - p * e.e2)};

    // Rotational dynamics, B&M ch. 3: J w_dot = -w x (J w) + M, solved for
    // w_dot with the Gamma coefficients (exact for the xz-symmetric inertia matrix).
    d.w_body = {g.g1 * p * q - g.g2 * q * r + g.g3 * l + g.g4 * n,
                g.g5 * p * r - g.g6 * (p * p - r * r) + m / g.Jy,
                g.g7 * p * q - g.g1 * q * r + g.g4 * l + g.g8 * n};
    return d;
}

Vec3 gravity_body(const Quat& att, double mass, double g0) {
    return rot_body_to_ned(att).transpose() * Vec3{0.0, 0.0, mass * g0};
}

// J = [[Jx, 0, -Jxz], [0, Jy, 0], [-Jxz, 0, Jz]]  (B&M ch. 3)
static Vec3 inertia_times(const Vec3& w, const MassProps& mp) {
    return {mp.Jx * w.x - mp.Jxz * w.z, mp.Jy * w.y, -mp.Jxz * w.x + mp.Jz * w.z};
}

Vec3 angular_momentum_ned(const State& x, const MassProps& mp) {
    return rot_body_to_ned(x.att) * inertia_times(x.w_body, mp);
}

double rotational_ke(const State& x, const MassProps& mp) {
    return 0.5 * x.w_body.dot(inertia_times(x.w_body, mp));
}

}  // namespace bwb
