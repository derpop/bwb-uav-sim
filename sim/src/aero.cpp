#include "bwb/aero.hpp"

constexpr double g = 9.81;
namespace bwb{

    AeroParams aerosonde_params(){
        AeroParams p;
        p.S = 0.55; //m^2
        p.b = 2.8956; // m
        p.c = 0.18994; //m
        p.S_prop = 0.2027; //m^2
        p.rho = 1.2682; //kg/m^3
        p.e = 0.9;
    //Motor
        p.k_motor = 80;
        p.k_T_p = 0;
        p.k_Omega = 0;
        p.C_prop = 1.0;
    //Stall Blend
        p.M = 50;
        p.alpha0 = 0.4712; //rad 
    //Longitudinal
        p.C_L_alpha = 3.45;
        p.C_L_0 = 0.28;
        p.C_L_q = 0;
        p.C_L_de = -0.36;
        p.C_D_p = 0.0437;
        p.C_D_q = 0;
        p.C_D_de = 0;
        p.C_m_0 = -0.02338;
        p.C_m_alpha = -0.38;
        p.C_m_q = -3.6;
        p.C_m_de = -0.5;
    //Lateral
        p.C_Y_0 = 0;
        p.C_Y_beta = -0.98;
        p.C_Y_p = 0;
        p.C_Y_r = 0;
        p.C_Y_da = 0;
        p.C_Y_dr = -0.17;

        p.C_l_0 = 0;
        p.C_l_beta = -0.12;
        p.C_l_p = -0.26;
        p.C_l_r = 0.14;
        p.C_l_da = 0.08;
        p.C_l_dr = 0.105;

        p.C_n_0 = 0;
        p.C_n_beta = 0.25;
        p.C_n_p = 0.022;
        p.C_n_r = -0.35;
        p.C_n_da = 0.06;
        p.C_n_dr = -0.032;
        return p;
    }
    double stall_sigma(double alpha, const AeroParams& p){
        const double A = std::exp(-p.M * (alpha - p.alpha0));
        const double B = std::exp(p.M * (alpha + p.alpha0));
        return (1 + A + B) / ((1+A) * (1+B));
    }

    double C_L_of_alpha(double alpha, const AeroParams& p){
        const double sigma = stall_sigma(alpha, p);
        return (1 - sigma) * (p.C_L_0 + p.C_L_alpha * alpha) + sigma * 
        (2 * std::copysign(1.0, alpha) * std::sin(alpha) * std::sin(alpha) * std::cos(alpha));
    }

    double C_D_of_alpha(double alpha, const AeroParams& p){
        const double AR = p.b * p.b / p.S;
        const double C_L_lin  = p.C_L_0 + p.C_L_alpha * alpha;
        return p.C_D_p + C_L_lin * C_L_lin / (M_PI * AR * p.e);
    }

    ForcesMoments forces_moments(const State& x, const Controls& u, const Vec3& wind_ned,
    const AeroParams& p, const MassProps& mass){
        ForcesMoments out;
        const Mat3 Rt = rot_body_to_ned(x.att).transpose();
        Vec3 f_gravity = Rt * Vec3(0, 0, mass.m * g);
        Vec3 wind_body = Rt * wind_ned;
        Vec3 V_body = x.vel_body - wind_body;
        const double ur = V_body.x;
        const double vr = V_body.y;
        const double wr = V_body.z;
        const double Va = V_body.magnitude();
        double alpha = 0;
        double beta = 0;
        Vec3 f_aero;
        Vec3 m_aero;
        if(Va > 1e-6){
            alpha = std::atan2(wr, ur);
            beta = std::asin(vr/ Va);
            const double P = x.omega.x;
            const double Q = x.omega.y;
            const double R = x.omega.z;
            const double p_hat = P * p.b / (2 * Va);
            const double qbar_S = 0.5 * p.rho * Va * Va * p.S;
            const double q_hat = Q * p.c / (2 * Va);
            const double r_hat = R * p.b / (2 * Va);
            const double C_L = C_L_of_alpha(alpha, p) + p.C_L_q * q_hat + p.C_L_de * u.de;
            const double C_D = C_D_of_alpha(alpha, p) + p.C_D_q * q_hat + p.C_D_de * u.de;
            const double L = qbar_S * C_L;
            const double D = qbar_S * C_D;
            const double fx = -D * std::cos(alpha) + L * std::sin(alpha);
            const double fz = -D * std::sin(alpha) - L * std::cos(alpha);
            const double fy = qbar_S * (p.C_Y_0 + p.C_Y_beta * beta + p.C_Y_p * p_hat
            + p.C_Y_r * r_hat + p.C_Y_da * u.da + p.C_Y_dr * u.dr);

            const double l_roll = qbar_S * p.b *(p.C_l_0 + p.C_l_beta * beta + p.C_l_p * p_hat
            + p.C_l_r * r_hat + p.C_l_da * u.da+ p.C_l_dr * u.dr);

            const double M_pitch =  qbar_S * p.c *(p.C_m_0 + p.C_m_alpha * alpha + 
                p.C_m_q * q_hat + p.C_m_de * u.de);

            const double N_yaw = qbar_S * p.b *(p.C_n_0 + p.C_n_beta * beta + p.C_n_p * p_hat
            + p.C_n_r * r_hat + p.C_n_da * u.da+ p.C_n_dr * u.dr);
            f_aero = Vec3(fx, fy, fz);
            m_aero = Vec3(l_roll, M_pitch, N_yaw);
        }
        const double F_prop = 0.5 * p.rho * p.S_prop * p.C_prop * ((p.k_motor * u.dt)*(p.k_motor * u.dt) - Va* Va);
        const double l_prop = -p.k_T_p * (p.k_Omega * u.dt)* (p.k_Omega * u.dt);
        out.force = f_gravity + f_aero + Vec3(F_prop, 0, 0);
        out.moment = m_aero + Vec3(l_prop, 0, 0);
        out.Va = Va;
        out.alpha = alpha;
        out.beta = beta;
        return out;
    }
}