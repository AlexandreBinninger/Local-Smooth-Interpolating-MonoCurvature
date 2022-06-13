#pragma once

#include "curve.h"
#include "../utils/misc.h"
#include "../utils/polygons.h"
#include "Clothoids.hh"
#include "../utils/increase.h"


class Clothoid_3arcs : public Curve 
{
public:
    G2lib::ClothoidCurve *curve_S0;
    G2lib::ClothoidCurve *curve_SM;
    G2lib::ClothoidCurve *curve_S1;

    double L0, LM, L1;
    

    Clothoid_3arcs(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const double &theta0, const double &theta1, const double &K0, const double &K1);

    void eval(const double &t, Eigen::Vector3d &result) const override;
    bool is_monotonous();
    double curvature_begin();
    double curvature_end();
    double curvature_middle_begin();
    double curvature_middle_end();
    bool curvature_change_sign();
    bool control_points_maxcurvature();
    void print_curvature_signs();
    bool monotonous_or_maxcurvature();
};

void curvature_modifier_clothoid_3arcs(const Eigen::MatrixXd &points, const Eigen::MatrixXd &tangents, const Eigen::VectorXd &ref_curvatures, const Eigen::VectorXi &curvature_constraints_index, const Eigen::VectorXi &no_constraints_index, Eigen::VectorXd &new_curvatures);