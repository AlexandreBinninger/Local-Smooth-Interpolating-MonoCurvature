#pragma once
#include "../global.h"
#include "misc.h"
// Alpha increasing functions
enum Increase_Type{
    Linear_incr,
    Max_Linear_incr
};

extern Increase_Type increase_function;

void increase(const double& K_0, const double& K_1, const double& alpha, double& new_K_0, double& new_K_1);
void increase(const double &K_0, const double &K_1, const double &K_2, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2);
void increase(const double &K_0, const double &K_1, const double &K_2, const double &K_3, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2, double &new_K_3);
void max_linear_increase(const double& K_0, const double& K_1, const double& alpha, double& new_K_0, double& new_K_1);
void max_linear_increase(const double &K_0, const double &K_1, const double &K_2, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2);
void max_linear_increase(const double &K_0, const double &K_1, const double &K_2, const double &K_3, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2, double &new_K_3);
void linear_increase(const double& K_0, const double& K_1, const double& alpha, double& new_K_0, double& new_K_1);
void linear_increase(const double &K_0, const double &K_1, const double &K_2, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2);
void linear_increase(const double &K_0, const double &K_1, const double &K_2, const double &K_3, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2, double &new_K_3);

