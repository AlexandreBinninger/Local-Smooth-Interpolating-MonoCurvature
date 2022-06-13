#pragma once

#include "../global.h"


static inline double sign(const double& a){return (double)((a>0)?+1:-1);}

static inline int pos_modulo(const int &a, const int &q)
{
    return (q + (a % q)) % q;
}

static double pos_atan2(const double &y, const double &x){ // result in [0, 2*pi]
    const double res = atan2(y, x);
    return res < 0 ? res+2*M_PI : res;
}

static inline double signed_angle2D(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2){ // in [-pi, +pi] 
    double angle_v1 = pos_atan2(v1(1), v1(0));
    double angle_v2 = pos_atan2(v2(1), v2(0));
    double result = angle_v2 - angle_v1;
    while (result < - M_PI){result += 2*M_PI;}
    while (result > M_PI){result -= 2*M_PI;}
    
    return result;
}

static inline void vector_from_angle(const double& theta, Eigen::Vector3d &result){
    result.setZero();
    result(0) = cos(theta);
    result(1) = sin(theta);
}

static inline double alignment(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2){
    Eigen::Vector3d v1_normalize = v1.normalized();
    Eigen::Vector3d v2_normalize = v2.normalized();
    return abs(v1_normalize(0) * v2_normalize(1) - v1_normalize(1)*v2_normalize(0));
}

static inline bool are_aligned(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, double approx_limit = EPSILON_APPROX){
    return abs(v1(0) * v2(1) - v1(1)*v2(0)) < approx_limit;
}

static inline double positive_main_angle(const double& angle){
    double result = angle;
    while (result > 2*M_PI){
        result -= 2*M_PI;
    }
    while (result < 0){
        result += 2*M_PI;
    }
    return result;
}

static inline double signed_main_angle(const double& angle){
    double result = angle;
    while (result > M_PI){
        result -= 2*M_PI;
    }
    while (result < -M_PI){
        result += 2*M_PI;
    }
    return result;
}