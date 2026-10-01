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
        x.att = bwb::euler_to_quat(bwb::Euler(deg(20), deg(30), deg(40)));
        auto out = bwb::forces_moments(x, c, no_wind, p, aero_mass);
        check_near(out.force, bwb::Vec3(-66.2175, 39.2270, 107.7753), 1e-3, "B: gravity, tilted");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        auto out = bwb::forces_moments(x, c, no_wind, p, aero_mass);
        check_near(out.Va, 25 , 1e-3, "Air Data Va");
        check_near(out.alpha, 0, 1e-3, "Air Data alpha");
        check_near(out.beta, 0, 1e-3, "Air Data beta");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        bwb::Vec3 wind(-5, 0, 0);
        auto out = bwb::forces_moments(x, c, wind, p, aero_mass);
        check_near(out.Va, 30 , 1e-3, "Headwind north");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        bwb::Vec3 wind(0, -5, 0);
        auto out = bwb::forces_moments(x, c, wind, p, aero_mass);
        check_near(out.Va, 25.4951 , 1e-3, "Crosswind east Va");
        check_near(out.beta, 0.19740, 1e-3, "Crosswind east beta");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(90)));
        bwb::Vec3 wind(0, -5, 0);
        auto out = bwb::forces_moments(x, c, wind, p, aero_mass);
        check_near(out.Va, 30 , 1e-3, "Heading east into Headwind east Va");
        check_near(out.beta, 0, 1e-3, "Heading east into Headwind east beta");
    }
    {
        bwb::State x;
        x.vel_body = bwb::Vec3(25, 0, 0);
        x.att = bwb::euler_to_quat(bwb::Euler(deg(0), deg(0), deg(0)));
        bwb::Vec3 wind(25, 0, 0);
        auto out = bwb::forces_moments(x, c, wind, p, aero_mass);
        check_near(out.Va, 0 , 1e-3, "No airspeed Va");
        check_near(out.alpha, 0, 1e-3, "No airspeed alpha");
        check_near(out.beta, 0, 1e-3, "No airspeed beta");
        if(!std::isfinite(out.force.x) || !std::isfinite(out.force.y) || !std::isfinite(out.force.z)){
            std::cerr << "Non-finite force detected" << std::endl;
            ++failures;
        }
    }

    std::cerr << "Total failures: " << failures << std::endl;
    return failures == 0? 0 : 1;
}