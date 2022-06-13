#include "clothoid_line.h" 
#include "clothoid_3arcs.h"


Clothoid_3arcs::Clothoid_3arcs(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const double &theta0, const double &theta1, const double &K0, const double &K1){
    G2lib::G2solve3arc g2solve3arc;
    int result_build = g2solve3arc.build(p0(0), p0(1), theta0, K0, p1(0), p1(1), theta1, K1);
    if (result_build<0){
        M_Assert(false, "no solution possible");
    }
    curve_S0 = new G2lib::ClothoidCurve(g2solve3arc.getS0());
    curve_SM = new G2lib::ClothoidCurve(g2solve3arc.getSM());
    curve_S1 = new G2lib::ClothoidCurve(g2solve3arc.getS1());

    L0 = curve_S0->length();
    LM = curve_SM->length();
    L1 = curve_S1->length();

    bounds = std::make_pair<double, double>(0., L0+LM+L1);
}


void Clothoid_3arcs::eval(const double &t, Eigen::Vector3d &result) const
{
    M_Assert(bounds.first <= t && t <= bounds.second, "Value inside the bounds");
    result.setZero();
    if (t <= L0)
    {
        double x, y;
        curve_S0->eval(t, x, y);
        result(0) = x;
        result(1) = y;
    }
    else if (t <= L0+LM)
    {
        const double new_param = t - L0;
        double x, y;
        curve_SM->eval(new_param, x, y);
        result(0) = x;
        result(1) = y;
    }
    else
    {
        const double new_param = t - L0 - LM;
        double x, y;
        curve_S1->eval(new_param, x, y);
        result(0) = x;
        result(1) = y;
    }
}

bool Clothoid_3arcs::is_monotonous(){
    const double dkappa0 = curve_S0->dkappa();
    const double dkappaM = curve_SM->dkappa();
    const double dkappa1 = curve_S1->dkappa();
    return (dkappa0 * dkappaM > 0) && (dkappaM * dkappa1 > 0);
}

double Clothoid_3arcs::curvature_begin(){
    return curve_S0->kappaBegin();
}

double Clothoid_3arcs::curvature_end(){
    return curve_S1->kappaEnd();
}

double Clothoid_3arcs::curvature_middle_begin(){
    return curve_SM->kappaBegin();
}

double Clothoid_3arcs::curvature_middle_end(){
    return curve_SM->kappaEnd();
}

bool Clothoid_3arcs::curvature_change_sign(){
    const double sign_begin = sign(curvature_begin());
    const double sign_end = sign(curvature_end());
    const double sign_middle_begin = sign(curvature_middle_begin());
    const double sign_middle_end = sign(curvature_middle_end());
    return (sign_begin * sign_end < 0) || (sign_begin * sign_middle_begin < 0) || (sign_begin * sign_middle_end < 0);
}

bool Clothoid_3arcs::control_points_maxcurvature(){
    // EPSILON APPROX NECESSARY FOR CIRCLES
    return (abs(curvature_begin()) + EPSILON_APPROX > abs(curvature_middle_begin())) && (abs(curvature_begin()) + EPSILON_APPROX > abs(curvature_middle_end())) && (abs(curvature_end()) + EPSILON_APPROX > abs(curvature_middle_begin())) && (abs(curvature_end()) + EPSILON_APPROX > abs(curvature_middle_end()));
}



void Clothoid_3arcs::print_curvature_signs(){
    const double sign_begin = sign(curvature_begin());
    const double sign_end = sign(curvature_end());
    const double sign_middle_begin = sign(curvature_middle_begin());
    const double sign_middle_end = sign(curvature_middle_end());

    std::cout << sign_begin << " " << sign_middle_begin << " " << sign_middle_end << " " << sign_end << std::endl;
}

bool Clothoid_3arcs::monotonous_or_maxcurvature(){
    double K0 = curvature_begin();
    double K1 = curvature_end();

    if (K0*K1 < 0){
        return is_monotonous();
    } else{
        return control_points_maxcurvature();
    }
}

void curvature_clotho_max_3arcs(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const double &theta0, const double &theta1, const double &K0, const double &K1, const bool &constrained0, const bool &constrained1, double &new_K0, double &new_K1)
{
    double alpha = 1;
    double step_alpha = 1;
    int count = 0;
    double alignment_max = 1e-4;
    double theta_obj, alignment, distance;

    new_K0 = K0;
    new_K1 = K1;

    Clothoid_3arcs *current_3arcs_curve = new Clothoid_3arcs(p0, p1, theta0, theta1, K0, K1);
    if (current_3arcs_curve->monotonous_or_maxcurvature())
    {
        return;
    }

    bool has_been_down = false;
    double current_new_K0, current_new_K1;
    while (count < Clothoid_Line::nb_curvature_iter)
    {
        increase(K0, K1, alpha, current_new_K0, current_new_K1);
        if (constrained0)
            current_new_K0 = K0;
        if (constrained1)
            current_new_K1 = K1;

        current_3arcs_curve = new Clothoid_3arcs(p0, p1, theta0, theta1, current_new_K0, current_new_K1);


        if (current_3arcs_curve->monotonous_or_maxcurvature())
        {
            has_been_down = true;
            step_alpha /= 2;
            alpha -= step_alpha;
            new_K0 = current_new_K0;
            new_K1 = current_new_K1;
        }
        else
        {
            alpha += step_alpha;
            if (!has_been_down)
            {
                step_alpha *= 2;
            }
        }
        count++;
    }
    
    if (alpha < 1)
    {
        new_K0 = K0;
        new_K1 = K1;
    }
}

void curvature_modifier_clothoid_3arcs(const Eigen::MatrixXd &points, const Eigen::MatrixXd &tangents, const Eigen::VectorXd &ref_curvatures, const Eigen::VectorXi &curvature_constraints_index, const Eigen::VectorXi &no_constraints_index, Eigen::VectorXd &new_curvatures)
{
    new_curvatures = ref_curvatures;

    int nb_iterations = new_curvatures.rows();
    for (int i = 0; i < nb_iterations; i++)
    {
        const Eigen::Vector3d p0 = points.row(pos_modulo(i, ref_curvatures.rows())).transpose();
        const Eigen::Vector3d t0 = tangents.row(pos_modulo(i, ref_curvatures.rows())).transpose();
        const double theta0 = atan2(t0(1), t0(0));
        const double K0 = ref_curvatures(pos_modulo(i, ref_curvatures.rows()));
        const bool constrained0 = !!curvature_constraints_index(pos_modulo(i, ref_curvatures.rows()));
        const bool no_constraint_0 = !!no_constraints_index(pos_modulo(i, ref_curvatures.rows())) > 0;

        const Eigen::Vector3d p1 = points.row(pos_modulo(i + 1, ref_curvatures.rows())).transpose();
        const Eigen::Vector3d t1 = tangents.row(pos_modulo(i + 1, ref_curvatures.rows())).transpose();
        const double theta1 = atan2(t1(1), t1(0));
        const double K1 = ref_curvatures(pos_modulo(i + 1, ref_curvatures.rows()));
        const bool constrained1 = !!curvature_constraints_index(pos_modulo(i + 1, ref_curvatures.rows()));
        const bool no_constraint_1 = !!no_constraints_index(pos_modulo(i + 1, ref_curvatures.rows())) > 0;

        if ((!constrained0 || !constrained1) && !(no_constraint_0) && !(no_constraint_1) && (abs(K0) > EPSILON_APPROX && abs(K1) > EPSILON_APPROX))
        {
            double new_K0, new_K1;
            curvature_clotho_max_3arcs(p0, p1, theta0, theta1, K0, K1, constrained0, constrained1, new_K0, new_K1);
            if (abs(new_curvatures(pos_modulo(i, ref_curvatures.rows()))) < abs(new_K0))
            {
                new_curvatures(pos_modulo(i, ref_curvatures.rows())) = new_K0;
            }
            if (abs(new_curvatures(pos_modulo(i + 1, ref_curvatures.rows()))) < abs(new_K1))
            {
                new_curvatures(pos_modulo(i + 1, ref_curvatures.rows())) = new_K1;
            }
        }
    }


    Eigen::MatrixXd new_ref_curvatures = new_curvatures;
    for (int i = 0; i < nb_iterations; i++)
    {
        const Eigen::Vector3d p0 = points.row(pos_modulo(i, new_ref_curvatures.rows())).transpose();
        const Eigen::Vector3d t0 = tangents.row(pos_modulo(i, new_ref_curvatures.rows())).transpose();
        const double theta0 = atan2(t0(1), t0(0));
        const double K0 = new_ref_curvatures(pos_modulo(i, new_ref_curvatures.rows()));
        const bool constrained0 = !!curvature_constraints_index(pos_modulo(i, new_ref_curvatures.rows()));
        const bool no_constraint_0 = !!no_constraints_index(pos_modulo(i, new_ref_curvatures.rows())) > 0;

        const Eigen::Vector3d p1 = points.row(pos_modulo(i + 1, new_ref_curvatures.rows())).transpose();
        const Eigen::Vector3d t1 = tangents.row(pos_modulo(i + 1, new_ref_curvatures.rows())).transpose();
        const double theta1 = atan2(t1(1), t1(0));
        const double K1 = new_ref_curvatures(pos_modulo(i + 1, new_ref_curvatures.rows()));
        const bool constrained1 = !!curvature_constraints_index(pos_modulo(i + 1, new_ref_curvatures.rows()));
        const bool no_constraint_1 = !!no_constraints_index(pos_modulo(i + 1, new_ref_curvatures.rows())) > 0;
        if ((!constrained0 || !constrained1) && ((no_constraint_0) || (no_constraint_1)) && !((no_constraint_0) && (no_constraint_1)) && (abs(K0) > EPSILON_APPROX && abs(K1) > EPSILON_APPROX))
        {
            double new_K0, new_K1;
            curvature_clotho_max_3arcs(p0, p1, theta0, theta1, K0, K1, constrained0, constrained1, new_K0, new_K1);
            if (abs(new_curvatures(pos_modulo(i, ref_curvatures.rows()))) < abs(new_K0))
            {
                new_curvatures(pos_modulo(i, ref_curvatures.rows())) = new_K0;
            }
            if (abs(new_curvatures(pos_modulo(i + 1, ref_curvatures.rows()))) < abs(new_K1))
            {
                new_curvatures(pos_modulo(i + 1, ref_curvatures.rows())) = new_K1;
            }
        }
    }
}