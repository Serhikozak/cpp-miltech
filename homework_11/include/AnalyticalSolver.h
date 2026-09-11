#pragma once
#include "interfaces/Common.hpp"
#include "interfaces/IBalisticSolver.h"
#include <cmath>

class AnalyticalSolver : public IBalisticSolver {
    
    public:
    float calcTimeOfFlight(float Z0, float V0, float m, float d, float l) override {
    float a = d * 9.81f * m - 2 * d * d * l * V0;
    float b = -3 * 9.81f * m * m + 3 * d * l * m * V0;
    float c = 6 * m * m * Z0;

    if (std::fabs(a) < 1e-12f)
        return std::sqrt(2.0f * Z0 / 9.81f);

    float p = -b * b / (3 * a * a);
    float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

    if (p >= 0)
        return std::sqrt(2.0f * Z0 / 9.81f);

    float arg = 3 * q / (2 * p) * std::sqrt(-3 / p);
    if (std::fabs(arg) > 1)
        return std::sqrt(2.0f * Z0 / 9.81f);

    float phi = std::acos(arg);
    float t   = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * (float)M_PI) / 3) - b / (3 * a);
    return t > 0 ? t : std::sqrt(2.0f * Z0 / 9.81f);
}

    float calcHDistance(float t, float V0, float m, float d, float lift) override {
        float base_terms = t * V0 - (d * std::pow(t, 2) * V0) / (2.0f * m);
    if (std::abs(V0) < 0.0001f) {
        // Базовий рух з урахуванням лінійного гальмування об повітря (члени t^1 та t^2)
        //return t * V0 - (d * std::pow(t, 2) * V0) / (2.0f * m);
        return 0.0f;
    }

    float lift2 = lift * lift;
    float lift4 = lift2 * lift2;
    float g  = 9.81f;

    // Член t^3 (перший дріб)
    float term3 = (std::pow(t, 3) * (6.0f * d * g * lift * m - 6.0f * d * d * (lift2 - 1.0f) * V0)) / (36.0f * m * m);

    // Член t^5 (другий дріб)
    float term5 = (std::pow(t, 5) * (3.0f * std::pow(d, 3) * g * std::pow(lift, 3) * m - 3.0f * std::pow(d, 4) * lift2 * (lift2 + 1.0f) * V0)) / (36.0f * (lift2 + 1.0f) * std::pow(m, 4));

    // Член t^4 (третій дріб)
    float term4 = (std::pow(t, 4) * (3.0f * std::pow(d, 3) * (lift2 + 1.0f) * lift2 * V0 + 6.0f * std::pow(d, 3) * (lift2 + 1.0f) * lift4 * V0 - 6.0f * d * d * g * (lift4 + lift2 + 1.0f) * lift * m)) / (36.0f * std::pow(lift2 + 1.0f, 2) * std::pow(m, 3));

    float final_dist = base_terms + term3 + term5 + term4;

    if (std::isnan(final_dist) || std::isinf(final_dist)) {
        return base_terms;
    }
    return final_dist;
}


    //return t * V0
        //- (d * std::pow(t, 2) * V0) / (2 * m)
        //+ (std::pow(t, 3) * (6 * d * 9.81f * l * m
           //- 6 * std::pow(d, 2) * (l2-1) * V0)) / (36 * std::pow(m, 2))
        //+ (std::pow(t, 4) * (-6 * std::pow(d, 2) * 9.81f * l * (1 + l2 + l4) * m
        //   + 3 * std::pow(d, 3) * l2 * (1 + l2) * V0
        //   + 6 * std::pow(d, 3) * l4 * (1 + l2) * V0))
        //  / (36 * std::pow(1 + l2, 2) * std::pow(m, 3))
        //+ (std::pow(t, 5) * (3 * std::pow(d, 3) * 9.81f * std::pow(l, 3) * m
        //   - 3 * std::pow(d, 4) * l2 * (1 + l2) * V0))
        //  / (36 * (1 + l2) * std::pow(m, 4));

    
        
};