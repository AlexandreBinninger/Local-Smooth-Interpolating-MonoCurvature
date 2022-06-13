#pragma once
#include "curve.h"
#include "../utils/misc.h"
#include "../utils/polygons.h"
#include "Clothoids.hh"
#include "../utils/polygons.h"
#include "../utils/increase.h"


class Clothoid_Line : public Curve 
{
public:

    G2lib::ClothoidCurve *curve_S0;
    G2lib::ClothoidCurve *curve_S1;
    Eigen::Vector3d p0, p1;
    double theta0, K0, theta1, K1, angle_max_0, angle_max_1;
    double dK0, dK1; 
    double L0, L1, L_Line;
    double phi, alignment;
    Eigen::Vector3d point_shift_0, point_shift_1;

    static double ALIGNMENT_LIMIT;
    static int nb_curvature_iter;
    static int nb_thetaobj_iter;

    void clotho_eval_parameters();
    Clothoid_Line(const Eigen::Vector3d &m_p0, const Eigen::Vector3d &m_p1, const double &m_theta0, const double &m_theta1, const double &m_K0, const double &m_K1);
    void eval(const double &t, Eigen::Vector3d &result) const override;
    void curvature2D(const double &t, double &signed_curvature, Eigen::Vector3d &direction) const override; 
    bool is_valid(); // return true if the resulting curve is G^2 smooth.
    friend std::ostream& operator <<(std::ostream& os, const Clothoid_Line& clothoid_line);
};

void evaluation_clotho0(const Eigen::Vector3d &p0, const double &theta0, const double& K0, const double& dK0, const double& L0, const double &t, Eigen::Vector3d &result);
void evaluation_clotho1(const Eigen::Vector3d &p1, const double &theta1, const double& K1, const double& dK1, const double& L1, const double &t, Eigen::Vector3d &result);
void evaluation_end_points(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const double &theta0, const double &theta1, const double &K0, const double &K1, const double theta_objective, Eigen::Vector3d &result0, Eigen::Vector3d &result1);

enum Curvature_Change_Type
{
    CHANGE_TANGENT,
    CHANGE_ARCS_3
};

enum Clothoid_Kind{
    TANGENT,
    ARCS_3
};

enum Arc_Curve_Type{
    CURVE_ARCS_3,
    CURVE_TANGENT,
    CURVE_LINE,
    CURVE_NO_CONSTRAINT
};

void find_right_theta_objective_intervalle_search(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const double &theta0, const double &theta1, const double& K0, const double& K1, double & theta_objective, double & alignment_result, double & distance_result);

void curvature_modifier_clothoid_line(const Eigen::MatrixXd &points, const Eigen::MatrixXd &tangents, const Eigen::VectorXd &ref_curvatures, const Eigen::VectorXi &curvature_constraints_index, const Eigen::VectorXi &no_constraints_index, Eigen::VectorXd &new_curvatures);
void curvature_modifier_clothoid_intersections(const Eigen::MatrixXd &points, const Eigen::MatrixXd &tangents, const Eigen::VectorXd &ref_curvatures, const Eigen::VectorXi &curvature_constraints_index, const Eigen::VectorXi &no_constraints_index, int nb_neighbors, Eigen::VectorXd &new_curvatures);
void curvature_modifier_clothoid_final(const Eigen::MatrixXd &points, const Eigen::MatrixXd &tangents, const Eigen::VectorXd &ref_curvatures, const Eigen::VectorXi &curvature_sign, const Eigen::VectorXi &curvature_constraints_index, const Eigen::VectorXi &no_constraints_index, Eigen::VectorXd &new_curvatures, Eigen::Vector<Arc_Curve_Type, Eigen::Dynamic> &arc_types);
