#include <iostream>
#include "bwb/rigid_body.hpp"

int main() {
        bwb::State start;
        bwb::Vec3 moment(0, 0, 0);
        bwb::Vec3 force(0, 0, 0);
        start.omega = bwb::Vec3(0.001, 3, 0.001);   // initial angular velocity
        bwb::MassProps mass_props(11.0, 0.8244, 1.135, 1.759, 0);   // Aerosonde
        bwb::Mat3 J(mass_props.Jx, 0, -mass_props.Jxz,
            0, mass_props.Jy, 0,
            -mass_props.Jxz, 0, mass_props.Jz);
        const double dt = 0.01;
        const int steps = 2000;   // 20 s
        bwb::State x = start;
        std::cout << "t,p,q,r,phi,theta,psi,T_drift" << std::endl;
        const double T0 = 0.5 * start.omega.dotProd(J * start.omega);
        for (int i = 0; i <= steps; i++) {
            bwb::Euler att = bwb::quat_to_euler(x.att);
            const double T = 0.5 * x.omega.dotProd(J * x.omega);
            const double t = i * dt;
            const double p = x.omega.x;
            const double q = x.omega.y;
            const double r = x.omega.z;
            const double phi = att.phi;
            const double theta = att.theta;
            const double psi = att.psi;
            std::cout << t << "," << p << "," << q << "," 
            << r << "," << phi << "," << theta << "," << psi 
            << ","<< (T - T0)/ T0 << std::endl;
            x = bwb::rk4_step(x, force, moment, mass_props, dt);
        }
    return 0;
}
