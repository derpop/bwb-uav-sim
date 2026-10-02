#include <iostream>
#include <cmath>
#include <string>
#include "bwb/math.hpp"
#include "bwb/rigid_body.hpp"
#include "bwb/trim.hpp"
#include "check.hpp"

double deg(double d) { return d * M_PI / 180.0; }


int main() {
    //Residual Test
    auto p = bwb::aerosonde_params();
    const bwb::MassProps aero_mass(13.5, 0.8244, 1.135, 1.759, 0.1204);
    {
        double alpha = 0.082321;
        double Va = 25;
        double residual = bwb::residual_of_alpha(alpha,Va, 0,p,aero_mass);
        check_near(residual, 0.0, 1e-3, "Residual test r = 0 at trim alpha");
        residual = bwb::residual_of_alpha(0,Va, 0,p,aero_mass);
        check_near(residual, 67.734, 1e-3, "Residual test r(0) = 67.7");
    }
    {
        double Va = 25;
        double gamma = 0;
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        check_true(result.status == bwb::TrimStatus::ok, "T1: status is ok");
        check_near(result.alpha, 0.082321, 1e-4, "T1 Trim result alpha matches expected value");
        check_near(result.ctrl.de ,-0.109324, 1e-4, "T1 Trim result elevator matches expected value");
        check_near(result.ctrl.dt, 0.333523, 1e-4, "T1 Trim result throttle matches expected value");
        double theta = bwb::quat_to_euler(result.x.att).theta;
        check_near(theta, 0.082321, 1e-4, "T1 Trim result theta matches expected value");
        check_near(result.x.vel_body.x, 24.915339, 1e-4, "T1 Trim result velocity x matches expected value");
        check_near(result.x.vel_body.y, 0.0, 1e-4, "T1 Trim result velocity y matches expected value");
        check_near(result.x.vel_body.z, 2.055700, 1e-4, "T1 Trim result velocity z matches expected value");
        bwb::ForcesMoments fm = bwb::forces_moments(result.x,result.ctrl,bwb::Vec3(), p, aero_mass);
        bwb::State x_dot = bwb::state_derivative(result.x, fm.force, fm.moment, aero_mass);
        check_near(x_dot.vel_body.x, 0.0, 1e-8, "T1 Trim result velocity derivative x matches expected value");
        check_near(x_dot.vel_body.y, 0.0, 1e-8, "T1 Trim result velocity derivative y matches expected value");
        check_near(x_dot.vel_body.z, 0.0, 1e-8, "T1 Trim result velocity derivative z matches expected value");
        check_near(x_dot.omega.x, 0.0, 1e-8, "T1 Trim result angular velocity derivative x matches expected value");
        check_near(x_dot.omega.y, 0.0, 1e-8, "T1 Trim result angular velocity derivative y matches expected value");
        check_near(x_dot.omega.z, 0.0, 1e-8, "T1 Trim result angular velocity derivative z matches expected value");
        check_near(x_dot.att.e0, 0.0, 1e-8, "T1 Trim result attitude derivative e0 matches expected value");
        check_near(x_dot.att.e1, 0.0, 1e-8, "T1 Trim result attitude derivative e1 matches expected value");
        check_near(x_dot.att.e2, 0.0, 1e-8, "T1 Trim result attitude derivative e2 matches expected value");
        check_near(x_dot.att.e3, 0.0, 1e-8, "T1 Trim result attitude derivative e3 matches expected value"); 
        check_near(x_dot.pos_ned.x, Va * std::cos(gamma), 1e-8, "T1 Trim result position derivative x matches expected value");
        check_near(x_dot.pos_ned.y, 0 , 1e-8, "T1 Trim result position derivative y matches expected value");
        check_near(x_dot.pos_ned.z, -Va * std::sin(gamma), 1e-8, "T1 Trim result position derivative z matches expected value");
    }
    {
        double Va = 25;
        double gamma = deg(5);
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        check_true(result.status == bwb::TrimStatus::ok, "T2: status is ok");
        check_near(result.alpha, 0.080579, 1e-4, "T2 Trim result alpha matches expected value");
        check_near(result.ctrl.de ,-0.108000, 1e-4, "T2 Trim result elevator matches expected value");
        check_near(result.ctrl.dt, 0.353937, 1e-4, "T2 Trim result throttle matches expected value");
        double theta = bwb::quat_to_euler(result.x.att).theta;
        check_near(theta, 0.167846, 1e-4, "T2 Trim result theta matches expected value");
        check_near(result.x.vel_body.x, 24.918882, 1e-4, "T2 Trim result velocity x matches expected value");
        check_near(result.x.vel_body.y, 0.0, 1e-4, "T2 Trim result velocity y matches expected value");
        check_near(result.x.vel_body.z, 2.012298, 1e-4, "T2 Trim result velocity z matches expected value");
        bwb::ForcesMoments fm = bwb::forces_moments(result.x,result.ctrl,bwb::Vec3(), p, aero_mass);
        bwb::State x_dot = bwb::state_derivative(result.x, fm.force, fm.moment, aero_mass);
        check_near(x_dot.vel_body.x, 0.0, 1e-8, "T2 Trim result velocity derivative x matches expected value");
        check_near(x_dot.vel_body.y, 0.0, 1e-8, "T2 Trim result velocity derivative y matches expected value");
        check_near(x_dot.vel_body.z, 0.0, 1e-8, "T2 Trim result velocity derivative z matches expected value");
        check_near(x_dot.omega.x, 0.0, 1e-8, "T2 Trim result angular velocity derivative x matches expected value");
        check_near(x_dot.omega.y, 0.0, 1e-8, "T2 Trim result angular velocity derivative y matches expected value");
        check_near(x_dot.omega.z, 0.0, 1e-8, "T2 Trim result angular velocity derivative z matches expected value");
        check_near(x_dot.att.e0, 0.0, 1e-8, "T2 Trim result attitude derivative e0 matches expected value");
        check_near(x_dot.att.e1, 0.0, 1e-8, "T2 Trim result attitude derivative e1 matches expected value");
        check_near(x_dot.att.e2, 0.0, 1e-8, "T2 Trim result attitude derivative e2 matches expected value");
        check_near(x_dot.att.e3, 0.0, 1e-8, "T2 Trim result attitude derivative e3 matches expected value");
        check_near(x_dot.pos_ned.x, Va * std::cos(gamma), 1e-8, "T2 Trim result position derivative x matches expected value");
        check_near(x_dot.pos_ned.y, 0 , 1e-8, "T2 Trim result position derivative y matches expected value");
        check_near(x_dot.pos_ned.z, -Va * std::sin(gamma), 1e-8, "T2 Trim result position derivative z matches expected value");
    }
    {
        double Va = 25;
        double gamma = deg(-3);
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        check_true(result.status == bwb::TrimStatus::ok, "T3: status is ok: gamma = -3 deg");
        check_near(result.alpha, 0.082798, 1e-4, "T3 Trim result alpha matches expected value");
        check_near(result.ctrl.de ,-0.109687, 1e-4, "T3 Trim result elevator matches expected value");
        check_near(result.ctrl.dt, 0.320616, 1e-4, "T3 Trim result throttle matches expected value");
        double theta = bwb::quat_to_euler(result.x.att).theta;
        check_near(theta, 0.030439, 1e-4, "T3 Trim result theta matches expected value");
        check_near(result.x.vel_body.x, 24.914354, 1e-4, "T3 Trim result velocity x matches expected value");
        check_near(result.x.vel_body.y, 0.0, 1e-4, "T3 Trim result velocity y matches expected value");
        check_near(result.x.vel_body.z, 2.067597, 1e-4, "T3 Trim result velocity z matches expected value");
    }
    {
        double Va = 35;
        double gamma = deg(0);
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        check_true(result.status == bwb::TrimStatus::ok, "T4: status is ok: gamma = 0 deg");
        check_near(result.alpha, 0.003490, 1e-4, "T4 Trim result alpha matches expected value");
        check_near(result.ctrl.de ,-0.049413, 1e-4, "T4 Trim result elevator matches expected value");
        check_near(result.ctrl.dt, 0.463821, 1e-4, "T4 Trim result throttle matches expected value");
        double theta = bwb::quat_to_euler(result.x.att).theta;
        check_near(theta, 0.003490, 1e-4, "T4 Trim result theta matches expected value");
        check_near(result.x.vel_body.x, 34.999787, 1e-4, "T4 Trim result velocity x matches expected value");
        check_near(result.x.vel_body.y, 0.0, 1e-4, "T4 Trim result velocity y matches expected value");
        check_near(result.x.vel_body.z, 0.122155, 1e-4, "T4 Trim result velocity z matches expected value");
    }
    {
        double Va = 18;
        double gamma = deg(0);
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        check_true(result.status == bwb::TrimStatus::ok, "T5: status is ok: gamma = 0 deg");
        check_near(result.alpha, 0.230594, 1e-4, "T5 Trim result alpha matches expected value");
        check_near(result.ctrl.de ,-0.222011, 1e-4, "T5 Trim result elevator matches expected value");
        check_near(result.ctrl.dt, 0.246126, 1e-4, "T5 Trim result throttle matches expected value");
        double theta = bwb::quat_to_euler(result.x.att).theta;
        check_near(theta, 0.230594, 1e-4, "T5 Trim result theta matches expected value");
        check_near(result.x.vel_body.x, 17.523556, 1e-4, "T5 Trim result velocity x matches expected value");
        check_near(result.x.vel_body.y, 0.0, 1e-4, "T5 Trim result velocity y matches expected value");
        check_near(result.x.vel_body.z, 4.113998, 1e-4, "T5 Trim result velocity z matches expected value");
    }
    {
        double Va = 14;
        double gamma = deg(0);
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        check_true(result.status == bwb::TrimStatus::no_bracket, "F1: Va 14 no_bracket");
    }
    {
        double Va = 25;
        double gamma = deg(-45);
        bwb::TrimResult result = bwb::trim(Va, gamma, p, aero_mass);
        check_true(result.status == bwb::TrimStatus::throttle_out_of_range, "F2: gamma  -45 deg throttle_out_of_range");
    }


    std::cout << "Total failures: " << failures << std::endl;
    return failures == 0 ? 0 : 1;

}