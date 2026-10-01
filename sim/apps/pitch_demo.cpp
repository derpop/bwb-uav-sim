#include <iostream>
#include "bwb/aero.hpp"


double deg(double d) { return d * M_PI / 180.0; }

int main() {
    auto p = bwb::aerosonde_params();
    bwb::State start;
    bwb::Controls ctrl;
    bwb::Vec3 no_wind;
    const bwb::MassProps aero_mass(13.5, 0.8244, 1.135, 1.759, 0.1204);
    start.vel_body = bwb::Vec3(25, 0, 0);
    start.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
    ctrl.de = 0;
    ctrl.dt = 0.5;
    const double dt = 0.01;
    const int steps = 1000;
    bwb::State x = start;
    std::cout << "t,Va,alpha,theta,q,altitude,u,north" << std::endl;
    for (int i = 0; i < steps; i++) {
            auto out = bwb::forces_moments(x, ctrl, no_wind, p, aero_mass);
            bwb::Euler att = bwb::quat_to_euler(x.att);
            const double t = i * dt;
            const double Va = out.Va;
            const double alpha = out.alpha;
            const double theta = att.theta;
            const double q = x.omega.y;
            const double altitude = -x.pos_ned.z;
            const double u = x.vel_body.x;
            const double north = x.pos_ned.x;
            std::cout << t << "," << Va << "," << alpha << "," << theta << "," 
            << q << ","<< altitude<< ","<< u<<","<< north<< std::endl;
            x = bwb::rk4_step(x, out.force, out.moment, aero_mass, dt);
    }
    return 0;
}
