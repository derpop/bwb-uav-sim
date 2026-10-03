#include "bwb/trim.hpp"
#include <iostream>
#include "check.hpp"
#include <cmath>


double deg(double d) { return d * M_PI / 180.0; }
namespace {
    double kick_altitude(double dt,const bwb::AeroParams p, const bwb::MassProps aero_mass){
        double Va = 25;
        double gamma = 0;
        int steps = std::lround(5.0/dt);
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        check_true(result.status == bwb::TrimStatus::ok, "Kick: trim status ok");
        auto deriv = [&](const bwb::State& s) { 
            bwb::ForcesMoments fm = bwb::forces_moments(s, result.ctrl, bwb::Vec3(), p, aero_mass); 
            return bwb::state_derivative(s, fm.force, fm.moment, aero_mass); 
        };
        bwb::State x = result.x;
        x.omega.y = 0.1;  // sudden nose-up pitch rate q, attitude stays at trim
        for (int i = 0; i < steps; ++i) {
            x = bwb::rk4_step(x, dt, deriv);    
        }
        return -x.pos_ned.z;
    }   
}

int main() {
    auto p = bwb::aerosonde_params();
    const bwb::MassProps aero_mass(13.5, 0.8244, 1.135, 1.759, 0.1204);    //Level 
    
    {
        double Va = 25;
        double gamma = 0;
        double dt = 0.01;
        int steps = 1000;
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        auto deriv = [&](const bwb::State& s) { 
            bwb::ForcesMoments fm = bwb::forces_moments(s, result.ctrl, bwb::Vec3(), p, aero_mass); 
            return bwb::state_derivative(s, fm.force, fm.moment, aero_mass); 
        };
        bwb::State x = result.x;
        check_true(result.status == bwb::TrimStatus::ok, "Level Trim did succeed");
        for (int i = 0; i < steps; ++i) {

            x = bwb::rk4_step(x, dt, deriv);
        }
        check_near(-x.pos_ned.z, -result.x.pos_ned.z , 1e-6, "Level: altitude change < 1e-6");
        check_near(x.pos_ned.x, Va * std::cos(gamma) * (steps * dt), 1e-6, "Level: went foward x");
        check_near(x.pos_ned.y, result.x.pos_ned.y, 1e-6, "Level: Didnt move horizontally: y");
        check_near(x.vel_body.x, result.x.vel_body.x, 1e-6, "Level: velocity x matches");
        check_near(x.vel_body.y, result.x.vel_body.y, 1e-6, "Level: velocity y matches");
        check_near(x.vel_body.z, result.x.vel_body.z, 1e-6, "Level: velocity z matches");
    }
    //Climb
    {
        double Va = 25;
        double gamma = deg(5);
        double dt = 0.01;
        int steps = 1000;
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        auto deriv = [&](const bwb::State& s) { 
            bwb::ForcesMoments fm = bwb::forces_moments(s, result.ctrl, bwb::Vec3(), p, aero_mass); 
            return bwb::state_derivative(s, fm.force, fm.moment, aero_mass); 
        };
        bwb::State x = result.x;
        check_true(result.status == bwb::TrimStatus::ok, "Climb Trim succeed check");
        for (int i = 0; i < steps; ++i) {
            x = bwb::rk4_step(x, dt, deriv);
        }
        check_near(-x.pos_ned.z, steps * dt * Va * std::sin(gamma) , 1e-6, "Climb: altitude changed");
        check_near(x.pos_ned.x, Va * std::cos(gamma) * (steps * dt), 1e-6, "Climb: went foward x");
        check_near(x.pos_ned.y, result.x.pos_ned.y, 1e-6, "Climb: Didnt move horizontally: y");
        check_near(x.vel_body.x, result.x.vel_body.x, 1e-6, "Climb: velocity x matches");
        check_near(x.vel_body.y, result.x.vel_body.y, 1e-6, "Climb: velocity y matches");
        check_near(x.vel_body.z, result.x.vel_body.z, 1e-6, "Climb: velocity z matches");
    }
        //Kick
    {
        double Va = 25;
        double gamma = 0;
        double dt = 0.01;
        int steps = 6000;
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        check_true(result.status == bwb::TrimStatus::ok, "Kick: trim status ok");
        auto deriv = [&](const bwb::State& s) { 
            bwb::ForcesMoments fm = bwb::forces_moments(s, result.ctrl, bwb::Vec3(), p, aero_mass); 
            return bwb::state_derivative(s, fm.force, fm.moment, aero_mass); 
        };
        bwb::State x = result.x;
        x.omega.y = 0.1;  // sudden nose-up pitch rate q, attitude stays at trim
        double start_alt = -x.pos_ned.z;
        double peak_alt = start_alt;
        for (int i = 0; i < steps; ++i) {
            x = bwb::rk4_step(x, dt, deriv);
            if (-x.pos_ned.z > peak_alt) peak_alt = -x.pos_ned.z;
        }
        //With stage rk4: converges with 1e-6 tolerance
        check_near(-x.pos_ned.z - start_alt, 0.7242238, 1e-6, "Kick: settles ~0.72 m higher");
        check_near(peak_alt - start_alt, 0.95, 0.05, "Kick: peak altitude between 0.9 and 1.0 m");
        check_near(x.vel_body.x, result.x.vel_body.x, 1e-4, "Kick: u back to trim");
        check_near(x.vel_body.z, result.x.vel_body.z, 1e-4, "Kick: w back to trim");
        check_near(x.omega.y, 0.0, 1e-4, "Kick: q back to zero");
    }
    //Check that dynamic forces work
    double h1 = kick_altitude(0.02, p, aero_mass);
    double h2 = kick_altitude(0.01, p, aero_mass);
    double h3 = kick_altitude(0.005, p, aero_mass);
    const double ratio = (h1 - h2)/(h2 - h3);
    std::cout << "Dynamic forces: ratio of altitude differences = " << ratio << std::endl;
    check_true(ratio > 12, "Dynamic forces: ratio of altitude differences > 12 ");
    return failures == 0 ? 0 : 1;
}