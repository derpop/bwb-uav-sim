#include "bwb/aero.hpp"
#include "check.hpp"
#include <iostream>

double deg(double d) { return d * M_PI / 180.0; }

int main() {
    auto p = bwb::aerosonde_params();
    std::cout << "Aerosonde parameters loaded successfully." << std::endl;
    check_near(bwb::stall_sigma(deg(0),p), 0, 1e-6, "stall_sigma check(0)");
    check_near(bwb::stall_sigma(deg(20),p), 0.0022, 1e-3, "stall_sigma check(20)");
    check_near(bwb::stall_sigma(deg(27),p), 0.5005, 1e-3, "stall_sigma check(27)");
    check_near(bwb::stall_sigma(deg(40),p), 1.0000, 1e-3, "stall_sigma check(40)");
    check_near(bwb::C_L_of_alpha(deg(0),p), 0.2800, 1e-3, "C_L_of_alpha check(0)");
    check_near(bwb::C_L_of_alpha(deg(20),p), 1.4815, 1e-3, "C_L_of_alpha check(20)");
    check_near(bwb::C_L_of_alpha(deg(27),p), 1.1358, 1e-3, "C_L_of_alpha check(27)");
    check_near(bwb::C_L_of_alpha(deg(40),p), 0.6330, 1e-3, "C_L_of_alpha check(40)");
    check_near(bwb::C_L_of_alpha(deg(-40),p), -0.6330, 1e-3, "C_L_of_alpha(-40)"); 
    check_near(bwb::C_D_of_alpha(deg(0),p), 0.0455, 1e-3, "C_D_of_alpha check(0)");
    check_near(bwb::C_D_of_alpha(deg(10),p), 0.0618, 1e-3, "C_D_of_alpha check(10)");

    const bwb::MassProps aero_mass(13.5, 0.8244, 1.135, 1.759, 0.1204);
    bwb::Vec3 no_wind;
    bwb::Controls c;

    //forces_moments checks
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        auto out = bwb::forces_moments(x, c, no_wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(-90.2544, 0, 71.4029), 1e-3, "A: full force, level");
        check_near(out.moment, bwb::Vec3(0, -0.9680, 0), 1e-3, "A: full moment, level");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(20), deg(30), deg(40)));
        auto out = bwb::forces_moments(x, c, no_wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(-156.4719, 39.2270, 46.7432), 1e-3, "B: full force, tilted");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25 * std::cos(deg(5)), 0, 25 * std::sin(deg(5)));
        bwb::Controls ctrl;
        ctrl.de = -0.1;
        x.att = bwb::euler_to_quat(bwb::Euler(0, 0, 0));
        auto out = bwb::forces_moments(x, ctrl, no_wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(-79.7999, 0, -2.5359), 1e-3, "C: AOA 5 degrees, elevator -0.1");
        check_near(out.moment, bwb::Vec3(0, -0.2708, 0), 1e-3, "C: AOA 5 degrees, elevator -0.1");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25 * std::cos(deg(5)), 25 * std::sin(deg(5)), 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        auto out = bwb::forces_moments(x, c, no_wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(-90.2544, -18.6412, 71.4029), 1e-3, "D: force when beta 5");
        check_near(out.moment, bwb::Vec3(-6.6095, -0.9680,13.7698), 1e-3, "D: moment when beta 5");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        x.omega = bwb::Vec3(0.5, 0.2, 0.1);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        auto out = bwb::forces_moments(x, c, no_wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(-90.2544, 0, 71.4029), 1e-3, "E: Rated A");
        check_near(out.moment, bwb::Vec3(-4.2400, -1.0812, -0.8772), 1e-3, "E: Rated A");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        bwb::Controls ctrl;
        ctrl.da = 0.1;
        ctrl.dr = 0.1;
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        auto out = bwb::forces_moments(x, ctrl, no_wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(-90.2544, -3.7055, 71.4029), 1e-3, "F: force, aileron + rudder 0.1");
        check_near(out.moment, bwb::Vec3(11.6764, -0.9680, 1.7672), 1e-3, "F: moment, aileron + rudder 0.1");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        bwb::Controls ctrl;
        ctrl.dt = 0.5;
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        auto out = bwb::forces_moments(x, ctrl, no_wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(115.3969, 0, 71.4029), 1e-3, "G: full force, throttle 0.5");
        check_near(out.moment, bwb::Vec3(0, -0.9680, 0), 1e-3, "G: full moment, throttle 0.5");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        bwb::Vec3 wind = bwb::Vec3(-5, 0, 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        auto out = bwb::forces_moments(x, c, wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(-129.9663,0, 44.5487), 1e-3, "H: Headwind north");
        check_near(out.moment, bwb::Vec3(0, -1.3939, 0), 1e-3, "H: Headwind north");
        check_near(out.Va, 30, 1e-3, "H: Headwind north");
    }
    {
        bwb::State x;
        bwb::Vec3 wind = bwb::Vec3(0, -5, 0);
        x.vel_body = bwb::Vec3(25, 0, 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        auto out = bwb::forces_moments(x, c, wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(-93.8646, -43.8528, 68.9616), 1e-3, "I: East Crosswind");
        check_near(out.moment, bwb::Vec3(-15.5486, -1.0067, 32.3929), 1e-3, "I: East Crosswind");
        check_near(out.Va, 25.4951, 1e-3, "I: East Crosswind");
        check_near(out.beta, 0.19740, 1e-3, "I: East Crosswind");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25 * std::cos(deg(40)), 0, 25 * std::sin(deg(40)));
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        auto out = bwb::forces_moments(x, c, no_wind, p, aero_mass);
        check_near(out.moment, bwb::Vec3(0, -11.9514, 0), 1e-3, "J: post-stall alpha 40");
        check_near(out.alpha, deg(40), 1e-3, "J: post-stall alpha 40");
    }
    std::cerr << "Total failures: " << failures << std::endl;
    return failures == 0? 0 : 1;
}