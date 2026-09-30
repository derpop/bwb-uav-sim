#pragma once
#include "bwb/rigid_body.hpp"

namespace bwb{
    struct AeroParams{
        //Geometry and air
        double S; //m^2
        double b; // m
        double c; //m
        double S_prop; //m^2
        double rho; //kg/m^3
        double e;
        //Motor
        double k_motor;
        double k_T_p;
        double k_Omega;
        double C_prop;
        //Stall Blend
        double M;
        double alpha0; //rad

        //Longitudinal
        double C_L_alpha;
        double C_L_0;
        double C_L_q;
        double C_L_de;
        double C_D_p;
        double C_D_q;
        double C_D_de;
        double C_m_0;
        double C_m_alpha;
        double C_m_q;
        double C_m_de;
        //Lateral
        double C_Y_0;
        double C_Y_beta;
        double C_Y_p;
        double C_Y_r;
        double C_Y_da;
        double C_Y_dr;

        double C_l_0;
        double C_l_beta;
        double C_l_p;
        double C_l_r;
        double C_l_da;
        double C_l_dr;

        double C_n_0;
        double C_n_beta;
        double C_n_p;
        double C_n_r;
        double C_n_da;
        double C_n_dr;

        
    };
    struct Controls{
        double de = 0; //Rad
        double da = 0; //Rad    
        double dr = 0; //Rad
        double dt = 0; // (0,1)
    };
    struct ForcesMoments{
        Vec3 force;
        Vec3 moment;
        double Va = 0;
        double alpha = 0; //Rad
        double beta = 0; //Rad
    };
    AeroParams aerosonde_params();
    double stall_sigma(double alpha, const AeroParams& p);
    double C_L_of_alpha(double alpha, const AeroParams& p);
    double C_D_of_alpha(double alpha, const AeroParams& p);
}