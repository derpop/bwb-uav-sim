#include <iostream>
#include "bwb/aero.hpp"
#include "bwb/trim.hpp"
#include <string>


double deg(double d) { return d * M_PI / 180.0; }

namespace {
    void print_row(double t, const bwb::State& x, const bwb::ForcesMoments& out) {
        bwb::Euler att = bwb::quat_to_euler(x.att);
        const double Va = out.Va;
        const double alpha = out.alpha;
        const double theta = att.theta;
        const double q = x.omega.y;
        const double altitude = -x.pos_ned.z;
        const double u = x.vel_body.x;
        const double north = x.pos_ned.x;
        std::cout << t << "," << Va << "," << alpha << "," << theta << "," 
            << q << ","<< altitude<< ","<< u<<","<< north<< std::endl;
    }
}

int main(int argc, char* argv[]) {
    auto p = bwb::aerosonde_params();
    bwb::State start;
    bwb::Controls ctrl;
    bwb::Vec3 no_wind;
    const bwb::MassProps aero_mass(13.5, 0.8244, 1.135, 1.759, 0.1204);
    start.vel_body = bwb::Vec3(25, 0, 0);
    start.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
    if(argc > 1 && std::string(argv[1]) == "--trim"){
        bwb::TrimResult trim_result = bwb::trim(25,0,p,aero_mass);
        if(trim_result.status != bwb::TrimStatus::ok){
            std::cerr << "Trim failed with status: " << static_cast<int>(trim_result.status) << std::endl;
            return 1;
        }
        start = trim_result.x;
        ctrl = trim_result.ctrl;
    }else{
        ctrl.de = 0;
        ctrl.dt = 0.5;
    }
    const double dt = 0.01;
    const int steps = 1000;
    bwb::State x = start;
    std::cout << "t,Va,alpha,theta,q,altitude,u,north" << std::endl;
    for (int i = 0; i < steps; i++) {
            auto out = bwb::forces_moments(x, ctrl, no_wind, p, aero_mass);
            print_row(i * dt, x, out);
            x = bwb::rk4_step(x, out.force, out.moment, aero_mass, dt);
    }
    bwb::ForcesMoments lastForce = bwb::forces_moments(x, ctrl, no_wind, p, aero_mass);
    print_row(steps * dt, x, lastForce);
    return 0;
}
