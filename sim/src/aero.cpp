#include "bwb/aero.hpp"
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
}