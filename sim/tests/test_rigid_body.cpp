// Verification tests for the rigid-body core. Each test compares the sim to a
// case with a known analytic answer or a conserved quantity, so a failure means
// a bug in the equations, not a modelling choice.
//
// Plain asserts with a tiny harness to avoid a test-framework dependency; CTest
// runs the executable and treats a non-zero exit as failure.

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "bwb/integrators.hpp"

using namespace bwb;

namespace {

int g_failures = 0;

void check_near(double got, double want, double tol, const std::string& what) {
    if (!(std::fabs(got - want) <= tol)) {
        std::printf("  FAIL %s: got %.12g, want %.12g (tol %.3g)\n", what.c_str(), got, want, tol);
        ++g_failures;
    }
}

const Wrench kNoWrench{};
constexpr double kPi = 3.14159265358979323846;

State simulate(State x, double t_end, double dt, const MassProps& mp,
               const std::function<Wrench(double, const State&)>& fn) {
    const InertiaGammas g = make_gammas(mp);
    const int n = static_cast<int>(std::lround(t_end / dt));
    for (int i = 0; i < n; ++i) x = rk4_step(x, i * dt, dt, mp, g, fn);
    return x;
}

// Euler <-> quaternion round trip, and the quaternion DCM against the textbook
// 3-2-1 Euler-angle DCM built from elementary rotations.
void test_attitude_conversions() {
    const Euler a{0.3, -0.4, 2.0};
    const Euler b = quat_to_euler(euler_to_quat(a));
    check_near(b.phi, a.phi, 1e-12, "round-trip phi");
    check_near(b.theta, a.theta, 1e-12, "round-trip theta");
    check_near(b.psi, a.psi, 1e-12, "round-trip psi");

    const double cf = std::cos(a.phi), sf = std::sin(a.phi);
    const double ct = std::cos(a.theta), st = std::sin(a.theta);
    const double cp = std::cos(a.psi), sp = std::sin(a.psi);
    // R_b^i for a 3-2-1 sequence (transpose of B&M's R_v^b).
    const double want[3][3] = {
        {ct * cp, sf * st * cp - cf * sp, cf * st * cp + sf * sp},
        {ct * sp, sf * st * sp + cf * cp, cf * st * sp - sf * cp},
        {-st, sf * ct, cf * ct}};
    const Mat3 r = rot_body_to_ned(euler_to_quat(a));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) check_near(r.m[i][j], want[i][j], 1e-12, "DCM entry");
}

// No force, no moment: the aircraft coasts in a straight line at constant attitude.
void test_coasting() {
    State x;
    x.v_body = {25.0, 0.0, 0.0};
    x.att = euler_to_quat({0.0, 0.0, kPi / 4});  // heading north-east
    const State y = simulate(x, 10.0, 0.01, kAerosonde, [](double, const State&) { return kNoWrench; });
    const double d = 250.0 / std::sqrt(2.0);
    check_near(y.p_ned.x, d, 1e-9, "coast north");
    check_near(y.p_ned.y, d, 1e-9, "coast east");
    check_near(y.p_ned.z, 0.0, 1e-9, "coast down");
}

// Free fall from rest with a pitched, rolled attitude: gravity is computed in body
// axes, rotated through the DCM twice, and must still give pd = g t^2 / 2 in NED.
void test_free_fall() {
    State x;
    x.att = euler_to_quat({0.2, 0.5, -1.0});
    const MassProps mp = kAerosonde;
    const State y = simulate(x, 3.0, 0.01, mp, [&](double, const State& s) {
        return Wrench{gravity_body(s.att, mp.mass), {}};
    });
    check_near(y.p_ned.z, 0.5 * 9.80665 * 9.0, 1e-9, "free-fall depth");
    check_near(y.p_ned.x, 0.0, 1e-9, "free-fall north drift");
}

// Steady roll at constant p about a principal axis: phi grows as p t.
void test_steady_roll() {
    MassProps mp = kAerosonde;
    mp.Jxz = 0.0;  // make x a principal axis so p stays constant
    State x;
    x.w_body = {0.5, 0.0, 0.0};
    const State y = simulate(x, 2.0, 0.001, mp, [](double, const State&) { return kNoWrench; });
    check_near(quat_to_euler(y.att).phi, 1.0, 1e-9, "roll angle after 2 s at 0.5 rad/s");
}

// Torque-free tumble with Jxz != 0: the angular momentum vector in NED and the
// rotational kinetic energy are exact invariants of the true solution.
void test_torque_free_invariants(const MassProps& mp, const char* name) {
    State x;
    x.att = euler_to_quat({0.1, 0.2, 0.3});
    x.w_body = {1.0, -2.0, 0.7};
    const Vec3 h0 = angular_momentum_ned(x, mp);
    const double t0 = rotational_ke(x, mp);
    const State y = simulate(x, 20.0, 0.001, mp, [](double, const State&) { return kNoWrench; });
    const Vec3 h1 = angular_momentum_ned(y, mp);
    const double rel_h = (h1 - h0).norm() / h0.norm();
    check_near(rel_h, 0.0, 1e-8, std::string(name) + " |dH|/|H|");
    check_near(rotational_ke(y, mp) / t0, 1.0, 1e-8, std::string(name) + " KE ratio");
    check_near(y.att.norm(), 1.0, 1e-12, std::string(name) + " quaternion norm");
}

// Intermediate-axis theorem: spin about the middle principal axis is unstable,
// so a tiny perturbation grows into a flip; spin about the largest axis is stable.
void test_intermediate_axis() {
    MassProps mp = kAerosonde;  // Jx < Jy < Jz, so y is the intermediate axis
    mp.Jxz = 0.0;
    auto max_off_axis = [&](Vec3 w0, int spin_axis) {
        State x;
        x.w_body = w0;
        const InertiaGammas g = make_gammas(mp);
        double worst = 0.0;
        for (int i = 0; i < 20000; ++i) {
            x = rk4_step(x, i * 1e-3, 1e-3, mp, g, [](double, const State&) { return kNoWrench; });
            const double c = spin_axis == 1 ? x.w_body.y : x.w_body.z;
            worst = std::fmax(worst, std::fabs(1.0 - std::fabs(c) / 3.0));
        }
        return worst;
    };
    const double y_spin = max_off_axis({1e-3, 3.0, 1e-3}, 1);
    const double z_spin = max_off_axis({1e-3, 1e-3, 3.0}, 2);
    if (!(y_spin > 0.5)) {
        std::printf("  FAIL intermediate-axis spin stayed put (max deviation %.3g)\n", y_spin);
        ++g_failures;
    }
    check_near(z_spin, 0.0, 1e-3, "major-axis spin deviation");
}

// Convergence order: halving dt should cut the error by ~2^4 = 16 for RK4.
void test_rk4_order() {
    const MassProps mp = kAerosonde;
    State x;
    x.att = euler_to_quat({0.1, 0.2, 0.3});
    x.v_body = {20.0, 1.0, -0.5};
    x.w_body = {1.0, -2.0, 0.7};
    auto fn = [&](double, const State& s) { return Wrench{gravity_body(s.att, mp.mass), {0.3, 0.0, 0.0}}; };
    const State ref = simulate(x, 2.0, 1e-4, mp, fn);
    const double e1 = (simulate(x, 2.0, 0.02, mp, fn).p_ned - ref.p_ned).norm();
    const double e2 = (simulate(x, 2.0, 0.01, mp, fn).p_ned - ref.p_ned).norm();
    const double ratio = e1 / e2;
    if (!(ratio > 12.0 && ratio < 20.0)) {
        std::printf("  FAIL RK4 error ratio %.2f, want ~16\n", ratio);
        ++g_failures;
    }
}

}  // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"attitude conversions", test_attitude_conversions},
        {"coasting", test_coasting},
        {"free fall", test_free_fall},
        {"steady roll", test_steady_roll},
        {"torque-free invariants (Aerosonde)", [] { test_torque_free_invariants(kAerosonde, "Aerosonde"); }},
        {"torque-free invariants (BWB v2)", [] { test_torque_free_invariants(kBwbV2, "BWB v2"); }},
        {"intermediate axis", test_intermediate_axis},
        {"RK4 order", test_rk4_order},
    };
    for (const auto& [name, fn] : tests) {
        const int before = g_failures;
        fn();
        std::printf("%s %s\n", g_failures == before ? "[ ok ]" : "[FAIL]", name);
    }
    return g_failures == 0 ? 0 : 1;
}
