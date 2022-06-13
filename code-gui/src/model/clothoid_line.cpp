#include "clothoid_line.h"
#include "clothoid_curve.h"

int Clothoid_Line::nb_thetaobj_iter = 16;
int Clothoid_Line::nb_curvature_iter = 16;
double Clothoid_Line::ALIGNMENT_LIMIT = 1e-2;

Clothoid_Line::Clothoid_Line(const Eigen::Vector3d &m_p0, const Eigen::Vector3d &m_p1, const double &m_theta0, const double &m_theta1, const double &m_K0, const double &m_K1)
{
    p0 = m_p0;
    p1 = m_p1;
    theta0 = m_theta0;
    theta1 = m_theta1;
    K0 = m_K0;
    K1 = m_K1;

    clotho_eval_parameters();
    if (L0 < EPSILON_APPROX)
    {
        L0 = 0;
        dK0 = 1;
    }
    if (L1 < EPSILON_APPROX)
    {
        L1 = 0;
        dK1 = 1;
    }

    curve_S0 = new G2lib::ClothoidCurve(p0(0), p0(1), theta0, K0, dK0, L0);
    curve_S1 = new G2lib::ClothoidCurve(p1(0), p1(1), theta1 + M_PI, -K1, -dK1, L1);
    evaluation_clotho0(p0, theta0, K0, dK0, L0, 1.0, point_shift_0);
    evaluation_clotho1(p1, theta1, K1, dK1, L1, 1.0, point_shift_1);

    L_Line = (point_shift_0 - point_shift_1).norm();

    bounds = std::make_pair<double, double>(0., L0 + L_Line + L1);
}

void clotho_line_params(const double &K, const double &theta_circle, const double &theta_line, double &L, double &dK, bool is_end = false)
{
    double lambda = (is_end) ? -sign(K) : sign(K);
    double tau = lambda * (theta_line - theta_circle);
    tau = positive_main_angle(tau);
    double R = sign(K) / K;
    L = 2 * tau * R;
    double A_squared = R * L;
    dK = -sign(K) * 1 / (A_squared);
}

void Clothoid_Line::clotho_eval_parameters()
{
    phi = 0;
    double distance_result = 0;
    bool is_possible;
    double distance;
    find_right_theta_objective_intervalle_search(p0, p1, theta0, theta1, K0, K1, phi, alignment, distance);
    clotho_line_params(K0, theta0, phi, L0, dK0);
    clotho_line_params(K1, M_PI + theta1, M_PI + phi, L1, dK1, true);
}

void Clothoid_Line::eval(const double &t, Eigen::Vector3d &result) const
{
    M_Assert(bounds.first <= t && t <= bounds.second, "Value inside the bounds");
    try
    {
        /* code */
        result.setZero();
        if (t <= L0)
        {
            if (t < EPSILON_APPROX)
            {
                result = p0;
            }
            else
            {
                curve_S0->eval(t, result(0), result(1));
            }
        }
        else if (t <= L0 + L_Line)
        {
            const double new_param = (t - L0) / (L_Line);
            result = point_shift_0 * (1 - new_param) + point_shift_1 * new_param;
        }
        else
        {
            if (L0 + L_Line + L1 - t < EPSILON_APPROX)
            {
                result = p1;
            }
            else
            {
                curve_S1->eval((L0 + L_Line + L1 - t), result(0), result(1));
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

bool Clothoid_Line::is_valid()
{
    const Eigen::Vector3d vector_angle(cos(phi), sin(phi), 0);
    return (alignment < ALIGNMENT_LIMIT) && (point_shift_1 - point_shift_0).dot(vector_angle);
}

void Clothoid_Line::curvature2D(const double &t, double &signed_curvature, Eigen::Vector3d &direction) const
{
    Curve::curvature2D(t, signed_curvature, direction);
    if (t <= L0)
    {
        if (t < EPSILON_APPROX)
        {
            signed_curvature = K0;
        }
        else
        {
            signed_curvature = curve_S0->kappa(t);
        }
    }
    else if (t <= L0 + L_Line)
    {
        signed_curvature = 0;
    }
    else
    {
        if (L0 + L_Line + L1 - t < EPSILON_APPROX)
        {
            signed_curvature = -K1;
        }
        else
        {
            signed_curvature = -curve_S1->kappa(L0 + L_Line + L1 - t);
        }
    }
    Eigen::Vector3d der;
    Curve::derivative(t, der);
    const double angle_rot = signed_curvature > 0 ? M_PI_2 : -M_PI_2;
    Eigen::MatrixXd Rot = Eigen::Matrix3d::Zero();
    rotationMatrix2D(angle_rot, Rot);
    direction = (Rot * der.normalized());
}

std::string print_clothoid(G2lib::ClothoidCurve clotho_curve)
{
    std::string result("");
    result += std::string("(x, y) = (") + std::to_string(clotho_curve.xBegin()) + std::string(", ") + std::to_string(clotho_curve.yBegin()) + std::string(") -- ");
    result += std::string("theta = ") + std::to_string(clotho_curve.thetaBegin()) + std::string(" -- ");
    result += std::string("K = ") + std::to_string(clotho_curve.kappaBegin()) + std::string(" -- ");
    result += std::string("dK = ") + std::to_string(clotho_curve.kappa_D(0)) + std::string(" -- ");
    result += std::string("L = ") + std::to_string(clotho_curve.length());
    return result;
}

std::string print_line(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1)
{
    std::string result("");
    result += std::string("(x, y) = (") + std::to_string(p0(0)) + std::string(", ") + std::to_string(p0(1)) + std::string(") --");
    result += std::string("(x, y) = (") + std::to_string(p1(0)) + std::string(", ") + std::to_string(p1(1)) + std::string(")");
    return result;
}

std::ostream &operator<<(std::ostream &os, const Clothoid_Line &clothoid_line)
{
    std::string str_clotho_0 = print_clothoid(*clothoid_line.curve_S0);
    std::string str_clotho_1 = print_clothoid(*clothoid_line.curve_S1);
    std::string str_line = print_line(clothoid_line.point_shift_0, clothoid_line.point_shift_1);
    os << str_clotho_0 << "\n"
       << str_line << "\n"
       << str_clotho_1;
    return os;
}

void evaluation_clotho0(const Eigen::Vector3d &p0, const double &theta0, const double &K0, const double &dK0, const double &L0, const double &t, Eigen::Vector3d &result)
{
    double X, Y;
    double length = L0 * t;
    if (L0 < EPSILON_APPROX)
    {
        result = p0;
        return;
    }
    G2lib::ClothoidCurve curve_clothoid(p0(0), p0(1), theta0, K0, dK0, L0);
    result.setZero();
    curve_clothoid.eval(length, X, Y);
    result(0) = X;
    result(1) = Y;
}

void evaluation_clotho1(const Eigen::Vector3d &p1, const double &theta1, const double &K1, const double &dK1, const double &L1, const double &t, Eigen::Vector3d &result)
{
    double X, Y;
    double length = L1 * t;
    if (L1 < EPSILON_APPROX)
    {
        result = p1;
        return;
    }

    G2lib::ClothoidCurve curve_clothoid(p1(0), p1(1), theta1 + M_PI, -K1, -dK1, L1);
    result.setZero();
    curve_clothoid.eval(length, X, Y);
    result(0) = X;
    result(1) = Y;
}

void evaluation_end_points(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const double &theta0, const double &theta1, const double &K0, const double &K1, const double theta_objective, Eigen::Vector3d &result0, Eigen::Vector3d &result1)
{
    Eigen::Vector3d vector_theta_objective;
    double L0, dK0, L1, dK1;
    vector_from_angle(theta_objective, vector_theta_objective);
    clotho_line_params(K0, theta0, theta_objective, L0, dK0);
    clotho_line_params(K1, M_PI + theta1, M_PI + theta_objective, L1, dK1, true);
    evaluation_clotho0(p0, theta0, K0, dK0, L0, 1.0, result0);
    evaluation_clotho1(p1, theta1, K1, dK1, L1, 1.0, result1);
}

void find_right_theta_objective_intervalle_search(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const double &theta0, const double &theta1, const double &K0, const double &K1, double &theta_objective, double &alignment_result, double &distance_result)
{
    const Eigen::Vector3d p0_to_p1 = p1 - p0;
    theta_objective = theta0;
    Eigen::Vector3d vector_theta_objective;
    int nb_theta_step = 2 * Clothoid_Line::nb_thetaobj_iter;
    double theta_step = sign(K0) * 2 * M_PI / nb_theta_step;

    const double ALIGNMENT_MAX_BEGINNING = 10;
    double alignment_min = ALIGNMENT_MAX_BEGINNING;
    double theta_best = theta_objective;
    distance_result = (p0 - p1).squaredNorm();

    Eigen::Vector3d result0, result1, joined;
    //determine first quadrant
    for (int i = 0; i < nb_theta_step; i++)
    {
        evaluation_end_points(p0, p1, theta0, theta1, K0, K1, theta_objective, result0, result1);
        vector_from_angle(theta_objective, vector_theta_objective);
        joined = result1 - result0;
        joined.normalize();
        const double distance_current = (result1 - result0).squaredNorm();
        if (joined.dot(vector_theta_objective) > 0 && alignment(joined, vector_theta_objective) < alignment_min)
        {
            alignment_min = alignment(joined, vector_theta_objective);
            theta_best = theta_objective;
            distance_result = distance_current;
        }
        theta_objective += theta_step;
    }

    // Then, we will make some dichotomic search (we have the right direction almost)
    theta_step = sign(K0) * M_PI / nb_theta_step;
    std::vector<int> search_range{-1, 1};
    for (int i = 0; i < Clothoid_Line::nb_thetaobj_iter; i++)
    {
        double theta_ref = theta_best;
        for (int j : search_range)
        {
            double current_theta_objective = theta_ref + j * theta_step;

            evaluation_end_points(p0, p1, theta0, theta1, K0, K1, current_theta_objective, result0, result1);
            vector_from_angle(current_theta_objective, vector_theta_objective);
            joined = result1 - result0;
            joined.normalize();
            const double distance_current = (result1 - result0).squaredNorm();
            if (joined.dot(vector_theta_objective) > 0 && alignment(joined, vector_theta_objective) < alignment_min)
            {
                alignment_min = alignment(joined, vector_theta_objective);
                theta_best = current_theta_objective;
                distance_result = distance_current;
            }
        }
        theta_step /= 2;
    }

    theta_objective = theta_best;
    alignment_result = alignment_min;
}

void curvature_clotho_max_line(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const double &theta0, const double &theta1, const double &K0, const double &K1, const bool &constrained0, const bool &constrained1, double &new_K0, double &new_K1)
{
    double alpha = 1;
    double step_alpha = 1;
    int count = 0;
    double min_distance = (p0 - p1).squaredNorm() * 2, min_alpha = 1;
    const double alignment_max = Clothoid_Line::ALIGNMENT_LIMIT;
    double theta_obj, alignment, distance;

    find_right_theta_objective_intervalle_search(p0, p1, theta0, theta1, K0, K1, theta_obj, alignment, distance);
    if (alignment < alignment_max)
    {
        double L0, dK0, L1, dK1;
        Eigen::Vector3d result0, result1, vector_from_theta_obj;
        evaluation_end_points(p0, p1, theta0, theta1, K0, K1, theta_obj, result0, result1);
        vector_from_angle(theta_obj, vector_from_theta_obj);
        if ((result1 - result0).dot(vector_from_theta_obj) > 0)
        {
            new_K0 = K0;
            new_K1 = K1;
            return;
        }
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

        find_right_theta_objective_intervalle_search(p0, p1, theta0, theta1, current_new_K0, current_new_K1, theta_obj, alignment, distance);

        if (alignment < alignment_max)
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

    increase(K0, K1, alpha, new_K0, new_K1);
    if (constrained0)
        new_K0 = K0;
    if (constrained1)
        new_K1 = K1;
    find_right_theta_objective_intervalle_search(p0, p1, theta0, theta1, new_K0, new_K1, theta_obj, alignment, distance);
    int adjust_count = 0;
    while (alignment > alignment_max && adjust_count < 100)
    {
        alpha += step_alpha;
        increase(K0, K1, alpha, new_K0, new_K1);
        if (constrained0)
            new_K0 = K0;
        if (constrained1)
            new_K1 = K1;
        find_right_theta_objective_intervalle_search(p0, p1, theta0, theta1, new_K0, new_K1, theta_obj, alignment, distance);
        adjust_count++;
    }

    if (alpha < 1)
    {
        new_K0 = K0;
        new_K1 = K1;
    }
}

void curvature_modifier_clothoid_line(const Eigen::MatrixXd &points, const Eigen::MatrixXd &tangents, const Eigen::VectorXd &ref_curvatures, const Eigen::VectorXi &curvature_constraints_index, const Eigen::VectorXi &no_constraints_index, Eigen::VectorXd &new_curvatures)
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
            curvature_clotho_max_line(p0, p1, theta0, theta1, K0, K1, constrained0, constrained1, new_K0, new_K1);
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
            curvature_clotho_max_line(p0, p1, theta0, theta1, K0, K1, constrained0, constrained1, new_K0, new_K1);
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

/* 
    Curvature modifier for intersections
*/

bool intersect_2_clotho(G2lib::ClothoidCurve clotho_0, G2lib::ClothoidCurve clotho_1)
{
    double L0 = clotho_0.length();
    double L1 = clotho_1.length();
    if (L0 < EPSILON_APPROX || L1 < EPSILON_APPROX)
    {
        return false;
    }
    return clotho_0.collision(clotho_1);
}

bool intersects(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, const double &theta0, const double &theta1, const double &theta2, const double &theta3, const double &K0, const double &K1, const double &K2, const double &K3)
{

    Clothoid_Line clotho_line_0(p0, p1, theta0, theta1, K0, K1);
    Clothoid_Line clotho_line_1(p2, p3, theta2, theta3, K2, K3);

    G2lib::ClothoidCurve clotho_0_0 = G2lib::ClothoidCurve(*clotho_line_0.curve_S0);
    G2lib::ClothoidCurve clotho_0_1 = G2lib::ClothoidCurve(*clotho_line_0.curve_S1);
    const Eigen::Vector3d pe_0_0 = clotho_line_0.point_shift_0;
    const Eigen::Vector3d pe_0_1 = clotho_line_0.point_shift_1;
    const Eigen::Vector3d v_0 = pe_0_1 - pe_0_0;
    const double theta_line_0 = atan2(v_0(1), v_0(0));
    const double L0 = v_0.norm();
    G2lib::LineSegment line_0_aux(pe_0_0(0), pe_0_0(1), theta_line_0, L0);
    G2lib::ClothoidCurve line_0(line_0_aux);

    G2lib::ClothoidCurve clotho_1_0 = G2lib::ClothoidCurve(*clotho_line_1.curve_S0);
    G2lib::ClothoidCurve clotho_1_1 = G2lib::ClothoidCurve(*clotho_line_1.curve_S1);
    const Eigen::Vector3d pe_1_0 = clotho_line_1.point_shift_0;
    const Eigen::Vector3d pe_1_1 = clotho_line_1.point_shift_1;
    const Eigen::Vector3d v_1 = pe_1_1 - pe_1_0;
    const double theta_line_1 = atan2(v_1(1), v_1(0));
    const double L1 = v_1.norm();
    G2lib::LineSegment line_1_aux(pe_1_0(0), pe_1_0(1), theta_line_1, L1);
    G2lib::ClothoidCurve line_1(line_1_aux);

    bool intersect_0 = intersect_2_clotho(clotho_0_0, clotho_1_0);
    bool intersect_1 = intersect_2_clotho(clotho_0_0, clotho_1_1);
    bool intersect_2 = intersect_2_clotho(clotho_0_0, line_1);
    bool intersect_3 = intersect_2_clotho(clotho_0_1, clotho_1_0);
    bool intersect_4 = intersect_2_clotho(clotho_0_1, clotho_1_1);
    bool intersect_5 = intersect_2_clotho(clotho_0_1, line_1);
    bool intersect_6 = intersect_2_clotho(line_0, clotho_1_0);
    bool intersect_7 = intersect_2_clotho(line_0, clotho_1_1);
    bool intersect_8 = intersect_2_clotho(line_0, line_1);

    return intersect_0 || intersect_1 || intersect_2 || intersect_3 || intersect_4 || intersect_5 || intersect_6 || intersect_7 || intersect_8;
}

bool intersects(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const double &theta0, const double &theta1, const double &theta2, const double &K0, const double &K1, const double &K2)
{

    Clothoid_Line clotho_line_0(p0, p1, theta0, theta1, K0, K1);
    Clothoid_Line clotho_line_1(p1, p2, theta1, theta2, K1, K2);

    G2lib::ClothoidCurve clotho_0_0 = G2lib::ClothoidCurve(*clotho_line_0.curve_S0);
    G2lib::ClothoidCurve clotho_0_1 = G2lib::ClothoidCurve(*clotho_line_0.curve_S1);
    const Eigen::Vector3d pe_0_0 = clotho_line_0.point_shift_0;
    const Eigen::Vector3d pe_0_1 = clotho_line_0.point_shift_1;
    const Eigen::Vector3d v_0 = pe_0_1 - pe_0_0;
    const double theta_line_0 = atan2(v_0(1), v_0(0));
    const double L0 = v_0.norm();
    G2lib::LineSegment line_0_aux(pe_0_0(0), pe_0_0(1), theta_line_0, L0);
    G2lib::ClothoidCurve line_0(line_0_aux);

    G2lib::ClothoidCurve clotho_1_0 = G2lib::ClothoidCurve(*clotho_line_1.curve_S0);
    G2lib::ClothoidCurve clotho_1_1 = G2lib::ClothoidCurve(*clotho_line_1.curve_S1);
    const Eigen::Vector3d pe_1_0 = clotho_line_1.point_shift_0;
    const Eigen::Vector3d pe_1_1 = clotho_line_1.point_shift_1;
    const Eigen::Vector3d v_1 = pe_1_1 - pe_1_0;
    const double theta_line_1 = atan2(v_1(1), v_1(0));
    const double L1 = v_1.norm();
    G2lib::LineSegment line_1_aux(pe_1_0(0), pe_1_0(1), theta_line_1, L1);
    G2lib::ClothoidCurve line_1(line_1_aux);

    bool intersect_0 = intersect_2_clotho(clotho_0_0, clotho_1_0);
    bool intersect_1 = intersect_2_clotho(clotho_0_0, clotho_1_1);
    bool intersect_2 = intersect_2_clotho(clotho_0_0, line_1);
    bool intersect_4 = intersect_2_clotho(clotho_0_1, clotho_1_1);
    bool intersect_5 = intersect_2_clotho(clotho_0_1, line_1);
    bool intersect_6 = intersect_2_clotho(line_0, clotho_1_0);
    bool intersect_7 = intersect_2_clotho(line_0, clotho_1_1);
    bool intersect_8 = intersect_2_clotho(line_0, line_1);

    return intersect_0 || intersect_1 || intersect_2 || intersect_4 || intersect_5 || intersect_6 || intersect_7 || intersect_8;
}

void curvature_clotho_max_intersection(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, const double &theta0, const double &theta1, const double &theta2, const double &theta3, const double &K0, const double &K1, const double &K2, const double &K3, const bool &constrained0, const bool &constrained1, const bool &constrained2, const bool &constrained3, double &new_K0, double &new_K1, double &new_K2, double &new_K3)
{

    if (!intersects(p0, p1, p2, p3, theta0, theta1, theta2, theta3, K0, K1, K2, K3))
    {
        new_K0 = K0;
        new_K1 = K1;
        new_K2 = K2;
        new_K3 = K3;
        return;
    }

    double alpha = 1;
    double step_alpha = 1;
    int count = 0;
    bool has_been_down = false;

    double current_new_K0, current_new_K1, current_new_K2, current_new_K3;
    while (count < Clothoid_Line::nb_curvature_iter)
    {
        increase(K0, K1, K2, K3, alpha, current_new_K0, current_new_K1, current_new_K2, current_new_K3);
        if (constrained0)
            current_new_K0 = K0;
        if (constrained1)
            current_new_K1 = K1;
        if (constrained2)
            current_new_K2 = K2;
        if (constrained3)
            current_new_K3 = K3;

        if (!intersects(p0, p1, p2, p3, theta0, theta1, theta2, theta3, current_new_K0, current_new_K1, current_new_K2, current_new_K3))
        {
            has_been_down = true;
            step_alpha /= 2;
            alpha -= step_alpha;
            new_K0 = current_new_K0;
            new_K1 = current_new_K1;
            new_K2 = current_new_K2;
            new_K3 = current_new_K3;
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
    increase(K0, K1, K2, K3, alpha, new_K0, new_K1, new_K2, new_K3);
    int adjust_count = 0;
    while (intersects(p0, p1, p2, p3, theta0, theta1, theta2, theta3, new_K0, new_K1, new_K2, new_K3) && adjust_count < 100)
    {
        alpha += step_alpha;
        increase(K0, K1, K2, K3, alpha, new_K0, new_K1, new_K2, new_K3);
        if (constrained0)
            new_K0 = K0;
        if (constrained1)
            new_K1 = K1;
        if (constrained2)
            new_K2 = K2;
        if (constrained3)
            new_K3 = K3;
        adjust_count++;
    }

    if (alpha < 1)
    {
        new_K0 = K0;
        new_K1 = K1;
        new_K2 = K2;
        new_K3 = K3;
    }
}

void curvature_clotho_max_intersection_3points(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const double &theta0, const double &theta1, const double &theta2, const double &K0, const double &K1, const double &K2, const bool &constrained0, const bool &constrained1, const bool &constrained2, double &new_K0, double &new_K1, double &new_K2)
{
    // We suppose p1 is the common point, i.e. we have (p0, p1) and (p1, p2)

    if (!intersects(p0, p1, p2, theta0, theta1, theta2, K0, K1, K2))
    {
        new_K0 = K0;
        new_K1 = K1;
        new_K2 = K2;
        return;
    }

    double alpha = 1;
    double step_alpha = 1;
    int count = 0;
    bool has_been_down = false;

    double current_new_K0, current_new_K1, current_new_K2;
    while (count < Clothoid_Line::nb_curvature_iter)
    {
        increase(K0, K1, K2, alpha, current_new_K0, current_new_K1, current_new_K2);
        if (constrained0)
            current_new_K0 = K0;
        if (constrained1)
            current_new_K1 = K1;
        if (constrained2)
            current_new_K2 = K2;

        if (!intersects(p0, p1, p2, theta0, theta1, theta2, current_new_K0, current_new_K1, current_new_K2))
        {
            has_been_down = true;
            step_alpha /= 2;
            alpha -= step_alpha;
            new_K0 = current_new_K0;
            new_K1 = current_new_K1;
            new_K2 = current_new_K2;
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
    increase(K0, K1, K2, alpha, new_K0, new_K1, new_K2);
    int adjust_count = 0;
    while (intersects(p0, p1, p2, theta0, theta1, theta2, new_K0, new_K1, new_K2) && adjust_count < 100)
    {
        alpha += step_alpha;
        increase(K0, K1, K2, alpha, new_K0, new_K1, new_K2);
        if (constrained0)
            new_K0 = K0;
        if (constrained1)
            new_K1 = K1;
        if (constrained2)
            new_K2 = K2;
        adjust_count++;
    }

    if (alpha < 1)
    {
        new_K0 = K0;
        new_K1 = K1;
        new_K2 = K2;
    }
}

void curvature_modifier_clothoid_intersections(const Eigen::MatrixXd &points, const Eigen::MatrixXd &tangents, const Eigen::VectorXd &ref_curvatures, const Eigen::VectorXi &curvature_constraints_index, const Eigen::VectorXi &no_constraints_index, int nb_neighbors, Eigen::VectorXd &new_curvatures)
{
    if (nb_neighbors < 0)
    {
        return;
    }
    int nb_points = points.rows();
    if (nb_neighbors > nb_points)
    {
        nb_neighbors = nb_points;
    }

    new_curvatures = ref_curvatures;
    for (int i = 0; i < nb_points; i++)
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
        if ((!constrained0 || !constrained1) && !(no_constraint_0 && no_constraint_1) && (abs(K0) > EPSILON_APPROX && abs(K1) > EPSILON_APPROX))
        {
            for (int j = -nb_neighbors; j < nb_neighbors; j++)
            {
                const int index = pos_modulo(i + j, nb_points);
                const int index_next = pos_modulo(i + j + 1, nb_points);
                if ((j != 0))
                {
                    const Eigen::Vector3d p2 = points.row(index).transpose();
                    const Eigen::Vector3d t2 = tangents.row(index).transpose();
                    const double theta2 = atan2(t2(1), t2(0));
                    const double K2 = ref_curvatures(index);
                    const bool constrained2 = !!curvature_constraints_index(index);
                    const bool no_constraint_2 = !!no_constraints_index(index) > 0;
                    if (abs(K2) > EPSILON_APPROX)
                    {
                        double new_K0 = K0, new_K1 = K1, new_K2 = K2;
                        // Detect if there is an intersection here: do the reduction max-linearly on four/three curvatures.
                        if (j == 1)
                        {
                            curvature_clotho_max_intersection_3points(p0, p1, p2, theta0, theta1, theta2, K0, K1, K2, constrained0, constrained1, constrained2, new_K0, new_K1, new_K2);
                        }
                        else if (j == -1)
                        {
                            curvature_clotho_max_intersection_3points(p2, p0, p1, theta2, theta0, theta1, K2, K0, K1, constrained2, constrained0, constrained1, new_K2, new_K0, new_K1);
                        }
                        else
                        {
                            const Eigen::Vector3d p3 = points.row(index_next).transpose();
                            const Eigen::Vector3d t3 = tangents.row(index_next).transpose();
                            const double theta3 = atan2(t3(1), t3(0));
                            const double K3 = ref_curvatures(index_next);
                            const bool constrained3 = !!curvature_constraints_index(index_next);
                            const bool no_constraint_3 = !!no_constraints_index(index_next) > 0;
                            if (abs(K3) > EPSILON_APPROX)
                            {

                                double new_K3 = K3;
                                // Test whether the control polygon crosses or not.
                                bool intersection_control_polygon = intersect_2_segments(p0.block(0, 0, 2, 1), p1.block(0, 0, 2, 1), p2.block(0, 0, 2, 1), p3.block(0, 0, 2, 1));
                                if (!intersection_control_polygon)
                                {
                                    curvature_clotho_max_intersection(p0, p1, p2, p3, theta0, theta1, theta2, theta3, K0, K1, K2, K3, constrained0, constrained1, constrained2, constrained3, new_K0, new_K1, new_K2, new_K3);
                                }
                                if (abs(new_curvatures(index_next)) < abs(new_K3))
                                {
                                    new_curvatures(index_next) = new_K3;
                                }
                            }
                        }
                        if (abs(new_curvatures(pos_modulo(i, ref_curvatures.rows()))) < abs(new_K0))
                        {
                            new_curvatures(pos_modulo(i, ref_curvatures.rows())) = new_K0;
                        }
                        if (abs(new_curvatures(pos_modulo(i + 1, ref_curvatures.rows()))) < abs(new_K1))
                        {
                            new_curvatures(pos_modulo(i + 1, ref_curvatures.rows())) = new_K1;
                        }
                        if (abs(new_curvatures(index)) < abs(new_K2))
                        {
                            new_curvatures(index) = new_K2;
                        }
                    }
                }
            }
        }
    }
}
