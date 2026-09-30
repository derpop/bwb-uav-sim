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
    check_near(bwb::C_L_of_alpha(deg(-40),p), -0.6330, 1e-3, "C_L_of_alpha检查(-40)"); 
    check_near(bwb::C_D_of_alpha(deg(0),p), 0.0455, 1e-3, "C_D_of_alpha check(0)");
    check_near(bwb::C_D_of_alpha(deg(10),p), 0.0618, 1e-3, "C_D_of_alpha check(10)");
    std::cerr << "Total failures: " << failures << std::endl;
    return failures == 0? 0 : 1;
}