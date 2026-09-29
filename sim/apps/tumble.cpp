// Demo: torque-free tumble of the Aerosonde, written to CSV for plotting.
// Spins mostly about the intermediate (pitch) axis to show the "tennis racket"
// flip, and logs the invariants so you can see the integrator holding them.
//
//   ./tumble > tumble.csv
//   python3 analysis/plot_tumble.py tumble.csv

#include <cstdio>

#include "bwb/integrators.hpp"

using namespace bwb;

int main() {
    MassProps mp = kAerosonde;
    mp.Jxz = 0.0;  // principal axes, so the flip is clean
    const InertiaGammas g = make_gammas(mp);

    State x;
    x.w_body = {0.01, 3.0, 0.01};  // rad/s

    const double dt = 1e-3, t_end = 30.0;
    const int n = static_cast<int>(t_end / dt);
    const double h0 = angular_momentum_ned(x, mp).norm();
    const double ke0 = rotational_ke(x, mp);

    std::printf("t,p,q,r,phi,theta,psi,dH_rel,dKE_rel\n");
    for (int i = 0; i <= n; ++i) {
        if (i % 10 == 0) {
            const Euler e = quat_to_euler(x.att);
            std::printf("%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3e,%.3e\n", i * dt,
                        x.w_body.x, x.w_body.y, x.w_body.z, e.phi, e.theta, e.psi,
                        angular_momentum_ned(x, mp).norm() / h0 - 1.0,
                        rotational_ke(x, mp) / ke0 - 1.0);
        }
        x = rk4_step(x, i * dt, dt, mp, g, [](double, const State&) { return Wrench{}; });
    }
    return 0;
}
