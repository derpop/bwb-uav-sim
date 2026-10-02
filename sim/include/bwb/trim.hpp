#pragma once

#include "bwb/aero.hpp"



namespace bwb {
    enum class TrimStatus { ok, no_bracket, throttle_out_of_range, not_run};
    struct TrimResult{
        State x;
        Controls ctrl;
        double alpha = 0;
        TrimStatus status = TrimStatus::not_run;
    };
    TrimResult trim(double Va, double gamma, const AeroParams& p, const MassProps& mass);
    
    double residual_of_alpha(double alpha, double Va, double gamma, const AeroParams& p, const MassProps& mass);

}