#include "spline.h"

// Initializers
Spline::Spline()
{
    controlPoints = std::unique_ptr<ControlPoints>(new ControlPoints);
    array_points = std::deque<Eigen::MatrixXd>();
    curves = std::deque<Curve *>();
    parameter_array = std::deque<double>();
    total_length = -1;
}

void Spline::get_Interpolator(const Eigen::Matrix3d &m_points, Interpolator **I)
{
    try
    {
        *I = new class Hybrid(m_points);
    }
    catch (const std::runtime_error &e)
    {
        // Hybrid interpolation is actually a line
        *I = new Line(m_points);
    }
}

void Spline::compute_3arcs_clothoid(const Eigen::MatrixXd &ordered_points, std::deque<Curve *> &new_curves)
{
    const int nb_points = ordered_points.rows();
    const int nb_curves = cycle ? nb_points : nb_points - 1;
    M_Assert(nb_points >= 3, "We must have at least 3 points to perform clothoid interpolation");

    // 1 - Detect the specific points without constraints (duplicate, end points if no cycle)
    Eigen::VectorXi no_constraints;
    get_no_constraints(ordered_points, no_constraints);

    // 2 - Get control polygon and compute the desired sign of the curvature: +1 if positive, -1 if negative, 0 if aligned
    Eigen::VectorXi curvature_sign;
    get_curvature_sign(ordered_points, no_constraints, curvature_sign);

    // 3 - Estimation of the curvature
    Eigen::MatrixXd ordered_tangents(nb_points, 3);
    Eigen::VectorXd ordered_curvatures(nb_points);
    estimate_curvature(ordered_points, no_constraints, curvature_sign, ordered_tangents, ordered_curvatures);

    // 4 - Get input constraints
    controlPoints->constraints->apply_constraints(ordered_tangents, ordered_curvatures);
    Eigen::VectorXi curv_constraints_index;
    controlPoints->constraints->get_curvature_constraints_index(curv_constraints_index);

    // 5 - Curvature modifier(s)
    Eigen::VectorXd modified_curvatures = ordered_curvatures;
    if (refine)
    curvature_modifier_clothoid_3arcs(ordered_points, ordered_tangents, ordered_curvatures, curv_constraints_index, no_constraints, modified_curvatures);

    // 6 - Curve Generation
    for (int i = 0; i < nb_curves; i++)
    {
        const int index = pos_modulo(i, ordered_points.rows());
        const int index_next = pos_modulo(i + 1, ordered_points.rows());
        const Eigen::Vector3d p0 = ordered_points.row(index).transpose();
        const Eigen::Vector3d p1 = ordered_points.row(index_next).transpose();
        const Eigen::Vector3d t0 = ordered_tangents.row(index).transpose();
        const Eigen::Vector3d t1 = ordered_tangents.row(index_next).transpose();
        const double theta_0 = atan2(t0(1), t0(0));
        const double theta_1 = atan2(t1(1), t1(0));
        const double K0 = modified_curvatures(index);
        const double K1 = modified_curvatures(index_next);
        const int K_sign_0 = curvature_sign(index);
        const int K_sign_1 = curvature_sign(index_next);

        if (no_constraints(index) && no_constraints(index_next))
        {
            new_curves.push_back(new LinearCurve(p0, p1));
        }
        else if (K_sign_0 == 0 && K_sign_1 == 0)
        {
            new_curves.push_back(new LinearCurve(p0, p1));
        }
        else if (K_sign_0 == 0)
        {
            new_curves.push_back(new Clothoid_3arcs(p0, p1, theta_0, theta_1, 0.0, K1));
        }
        else if (K_sign_1 == 0)
        {
            new_curves.push_back(new Clothoid_3arcs(p0, p1, theta_0, theta_1, K0, 0.0));
        }
        else
        {
            new_curves.push_back(new Clothoid_3arcs(p0, p1, theta_0, theta_1, K0, K1));
        }
    }
}

void Spline::compute_clothoid_line(const Eigen::MatrixXd &ordered_points, std::deque<Curve *> &new_curves)
{
    const int nb_points = ordered_points.rows();
    const int nb_curves = cycle ? nb_points : nb_points - 1;
    M_Assert(nb_points >= 3, "We must have at least 3 points to perform clothoid interpolation");

    // 1 - Detect the specific points without constraints (duplicate, end points if no cycle)
    Eigen::VectorXi no_constraints;
    get_no_constraints(ordered_points, no_constraints);

    // 2 - Get control polygon and compute the desired sign of the curvature: +1 if positive, -1 if negative, 0 if aligned
    Eigen::VectorXi curvature_sign;
    get_curvature_sign(ordered_points, no_constraints, curvature_sign);

    // 3 - Estimation of the curvature
    Eigen::MatrixXd ordered_tangents(nb_points, 3);
    Eigen::VectorXd ordered_curvatures(nb_points);
    estimate_curvature(ordered_points, no_constraints, curvature_sign, ordered_tangents, ordered_curvatures);

    // 4 - Get input constraints
    controlPoints->constraints->apply_constraints(ordered_tangents, ordered_curvatures);
    Eigen::VectorXi curv_constraints_index;
    controlPoints->constraints->get_curvature_constraints_index(curv_constraints_index);

    // 5 - Curvature modifier(s)
    Eigen::VectorXd modified_curvatures = ordered_curvatures;
    if (refine)
    curvature_modifier_clothoid_line(ordered_points, ordered_tangents, ordered_curvatures, curv_constraints_index, no_constraints, modified_curvatures);
    if (intersection_detection)
    {
        // two passes are necessary for double intersections
        ordered_curvatures = modified_curvatures;
        if (refine)
        curvature_modifier_clothoid_intersections(ordered_points, ordered_tangents, ordered_curvatures, curv_constraints_index, no_constraints, neighbors_intersection, modified_curvatures);
        ordered_curvatures = modified_curvatures;
        if (refine)
        curvature_modifier_clothoid_intersections(ordered_points, ordered_tangents, ordered_curvatures, curv_constraints_index, no_constraints, neighbors_intersection, modified_curvatures);
    }

    // 6 - Curve Generation
    for (int i = 0; i < nb_curves; i++)
    {
        const int index = pos_modulo(i, ordered_points.rows());
        const int index_next = pos_modulo(i + 1, ordered_points.rows());
        const Eigen::Vector3d p0 = ordered_points.row(index).transpose();
        const Eigen::Vector3d p1 = ordered_points.row(index_next).transpose();
        const Eigen::Vector3d t0 = ordered_tangents.row(index).transpose();
        const Eigen::Vector3d t1 = ordered_tangents.row(index_next).transpose();
        const double theta_0 = atan2(t0(1), t0(0));
        const double theta_1 = atan2(t1(1), t1(0));
        const double K0 = modified_curvatures(index);
        const double K1 = modified_curvatures(index_next);
        const int K_sign_0 = curvature_sign(index);
        const int K_sign_1 = curvature_sign(index_next);
        if (no_constraints(index) && no_constraints(index_next))
        {
            new_curves.push_back(new LinearCurve(p0, p1));
        }
        else if (K_sign_0 == 0 && K_sign_1 == 0)
        {
            new_curves.push_back(new LinearCurve(p0, p1));
        }
        else if (K_sign_0 == 0)
        {
            new_curves.push_back(new Clothoid_3arcs(p0, p1, theta_0, theta_1, 0.0, K1));
        }
        else if (K_sign_1 == 0)
        {
            new_curves.push_back(new Clothoid_3arcs(p0, p1, theta_0, theta_1, K0, 0.0));
        }
        else
        {
            try
            {
                Clothoid_Line *curve_clothoid = new Clothoid_Line(p0, p1, theta_0, theta_1, K0, K1);
                if (use_correction_CLC && !curve_clothoid->is_valid()){
                    new_curves.push_back(new Clothoid_3arcs(p0, p1, theta_0, theta_1, K0, K1));
                } else{
                    new_curves.push_back(curve_clothoid);
                }            
            }
            catch (const std::exception &e)
            {
                new_curves.push_back(new LinearCurve(p0, p1));
            }
        }
    }
}

void Spline::compute_curves(const Eigen::MatrixXd &ordered_points, std::deque<Curve *> &new_curves)
{
    // 0 - Very specific cases
    if (ordered_points.rows() == 0)
    {
        new_curves = std::deque<Curve *>();
        return;
    }
    else if (ordered_points.rows() == 1)
    {
        const Eigen::VectorXd p = ordered_points.row(0).transpose();
        new_curves = std::deque<Curve *>();
        new_curves.push_back(new LinearCurve(p, p));
        return;
    }
    else if (ordered_points.rows() == 2)
    {
        const Eigen::VectorXd p0 = ordered_points.row(0).transpose();
        const Eigen::VectorXd p1 = ordered_points.row(1).transpose();
        new_curves = std::deque<Curve *>();
        new_curves.push_back(new LinearCurve(p0, p1));
        if (cycle)
            new_curves.push_back(new LinearCurve(p1, p0));
        return;
    }

    try
    {
        switch (spline_type)
        {
        case Curve_3Arcs_Clothoid:
            compute_3arcs_clothoid(ordered_points, new_curves);
            break;
        case Curve_Line_Clothoid:
            compute_clothoid_line(ordered_points, new_curves);
            break;
        default:
            break;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        Eigen::MatrixXd cp;
        controlPoints->get_points(cp);
        M_Log(cp, "Control Points:\n");
    }
}

/* 
    -----------------------
    Control Points handlers
    -----------------------
*/
// Initializer if there is no point already

void Spline::initialize_control_points(const Eigen::MatrixXd &m_points)
{
    M_Assert(controlPoints->empty(), "This function is to set spline when we know there is no control points already!");
    M_Assert(curves.size() == 0, "If there is no control point, there should be no curves neither.");
    M_Assert(array_points.size() == 0, "If there is no control point, there should be no line points neither.");
    controlPoints->add_points(m_points);
    std::deque<Curve *> new_curves;
    const int nb_points = m_points.rows();
    Eigen::MatrixXd curve_control_points = m_points;
    compute_curves(curve_control_points, new_curves);
    curves = new_curves;
}

void Spline::recompute_curves()
{
    total_length = -1;
    Eigen::MatrixXd old_points;
    Eigen::MatrixXi old_constraints_index;
    Eigen::MatrixXd old_tangents_constraints;
    Eigen::VectorXd old_curvatures_constraints;
    controlPoints->get_points(old_points);
    controlPoints->constraints->dump(old_constraints_index, old_tangents_constraints, old_curvatures_constraints);
    controlPoints->clear();
    curves.clear();
    array_points.clear();
    parameter_array.clear();
    controlPoints->add_points(old_points);
    controlPoints->constraints->reset(old_constraints_index, old_tangents_constraints, old_curvatures_constraints);
    std::deque<Curve *> new_curves;
    compute_curves(old_points, new_curves);
    curves = new_curves;
    recompute_spline();
    get_total_length();
}

void Spline::add_control_points(const Eigen::MatrixXd &m_points, int index)
{
    /* The index is the index of the point after which there is an insertion.
    If index==-1, the insertion is done after the last point.*/
    Eigen::MatrixXd current_points;
    Eigen::MatrixXd new_points;
    if (controlPoints->empty())
    {
        new_points = m_points;
    }
    else
    {
        controlPoints->get_points(current_points);
        new_points = Eigen::MatrixXd::Zero(current_points.rows() + m_points.rows(), 3);
        if (index == -1)
        {
            new_points.block(0, 0, current_points.rows(), 3) = current_points;
            new_points.block(current_points.rows(), 0, m_points.rows(), 3) = m_points;
        }
        else
        {
            new_points.block(0, 0, index, 3) = current_points.block(0, 0, index, 3);
            new_points.block(index, 0, m_points.rows(), 3) = m_points;
            new_points.block(index + m_points.rows(), 0, current_points.rows() - index, 3) = current_points.block(index, 0, current_points.rows() - index, 3);
        }
    }
    set_control_points(new_points);
}

void Spline::set_control_points(const Eigen::MatrixXd &m_points)
{
    hardreset();
    initialize_control_points(m_points);
    recompute_spline();
}

void Spline::remove_control_point(int index)
{
    int previous_number_of_points = controlPoints->size();
    M_Assert(index >= 0 && index < previous_number_of_points, "Index out of bounds for removing control point.");
    controlPoints->remove_point(index);
    recompute_curves();
}

void Spline::move_control_point(const Eigen::Vector3d &m_points, int index)
{
    int previous_number_of_points = controlPoints->size();
    M_Assert(index >= 0 && index < previous_number_of_points, "Index out of bounds for moving control point.");
    controlPoints->move_point(m_points, index);
    recompute_curves();
}

int Spline::size_control_points()
{
    return controlPoints->size();
}

void Spline::softreset()
{
    recompute_curves();
}

void Spline::hardreset()
{
    controlPoints = std::unique_ptr<ControlPoints>(new ControlPoints);
    array_points = std::deque<Eigen::MatrixXd>();
    curves = std::deque<Curve *>();
    parameter_array = std::deque<double>();
    total_length = -1;
}

// Internal Spline Computation
// Recompute entirely the spline
void Spline::recompute_spline()
{
    array_points = std::deque<Eigen::MatrixXd>();
    parameter_array = std::deque<double>();

    double curve_length = get_total_length();
    const double step_size = curve_length / (nb_points_per_curve * curves.size());
    double current_param = 0;
    int current_index_curve = 0;
    std::vector<Eigen::VectorXd> point_curves;
    while (current_param < curve_length)
    {
        int new_index_curve = -1;
        Eigen::VectorXd current_point;
        eval(current_param, current_point, new_index_curve);

        if (new_index_curve > current_index_curve)
        {
            Eigen::MatrixXd new_points;
            while (new_index_curve > current_index_curve + 1)
            {
                current_index_curve += 1;
                array_points.push_back(new_points);
            }
            current_index_curve += 1;
            new_points = Eigen::MatrixXd(point_curves.size(), 3);
            for (int i = 0; i < point_curves.size(); i++)
            {
                new_points.row(i) = point_curves[i].transpose();
            }
            point_curves.clear();
            point_curves.push_back(current_point);
            parameter_array.push_back(current_param);
            array_points.push_back(new_points);
        }
        else
        {
            point_curves.push_back(current_point);
            parameter_array.push_back(current_param);
        }
        current_param += step_size;
    }
    Eigen::MatrixXd new_points = Eigen::MatrixXd(point_curves.size(), 3);
    for (int i = 0; i < point_curves.size(); i++)
    {
        new_points.row(i) = point_curves[i].transpose();
    }
    array_points.push_back(new_points);

    compute_control_tangents();
    compute_control_curvatures();
}

void Spline::compute_control_tangents()
{
    const int nb_ctrl_p = controlPoints->size();
    const int nb_curves = curves.size();
    Eigen::MatrixXd tangents = Eigen::MatrixXd::Zero(nb_ctrl_p, 3);

    for (int i = 0; i < nb_ctrl_p; i++)
    {
        if (cycle || (i >= 1 && i < nb_ctrl_p - 1))
        {
            Eigen::Vector3d res_tan;
            Curve *cur_curve_prec = curves[pos_modulo((i - 1), nb_curves)];
            Curve *cur_curve_next = curves[pos_modulo(i, nb_curves)];
            cur_curve_prec->derivative(cur_curve_prec->get_upper_bound() - GlobalState::get_dt(), res_tan);
            tangents.row(i) += res_tan.transpose();
            cur_curve_next->derivative(cur_curve_next->get_lower_bound(), res_tan);
            tangents.row(i) += res_tan.transpose();
            tangents.row(i).normalize();
        }
        else
        {
            if (i == 0)
            {
                Eigen::Vector3d res_tan;
                Curve *cur_curve = curves[0];
                cur_curve->derivative(cur_curve->get_lower_bound(), res_tan);
                tangents.row(i) = res_tan.transpose().normalized();
            }
            else if (i == nb_ctrl_p - 1)
            {
                Eigen::Vector3d res_tan;
                Curve *cur_curve = curves[nb_curves - 1];
                cur_curve->derivative(cur_curve->get_upper_bound() - GlobalState::get_dt(), res_tan);
                tangents.row(i) = res_tan.transpose().normalized();
            }
        }
    }

    controlPoints->set_tangents(tangents);
}

void Spline::get_control_tangents(Eigen::MatrixXd &tangents)
{
    controlPoints->get_tangents(tangents);
}

void Spline::compute_control_curvatures()
{
    const int nb_ctrl_p = controlPoints->size();
    const int nb_curves = curves.size();
    Eigen::VectorXd curvatures = Eigen::VectorXd::Zero(nb_ctrl_p);

    for (int i = 0; i < nb_ctrl_p; i++)
    {
        if (cycle || (i >= 1 && i < nb_ctrl_p - 1))
        {
            double cur_curvature;
            Eigen::Vector3d direction;
            Curve *cur_curve_prec = curves[pos_modulo((i - 1), nb_curves)];
            Curve *cur_curve_next = curves[pos_modulo(i, nb_curves)];
            cur_curve_prec->curvature2D(cur_curve_prec->get_upper_bound() - 2 * GlobalState::get_dt(), cur_curvature, direction);
            curvatures(i) += cur_curvature;
            cur_curve_next->curvature2D(cur_curve_next->get_lower_bound() + 2 * GlobalState::get_dt(), cur_curvature, direction);
            curvatures(i) += cur_curvature;
            curvatures(i) /= 2;
        }
        else
        {
            if (i == 0)
            {
                double cur_curvature;
                Eigen::Vector3d direction;
                Curve *cur_curve = curves[0];
                cur_curve->curvature2D(cur_curve->get_lower_bound() + 2 * GlobalState::get_dt(), cur_curvature, direction);
                curvatures(i) = cur_curvature;
            }
            else if (i == nb_ctrl_p - 1)
            {
                double cur_curvature;
                Eigen::Vector3d direction;
                Curve *cur_curve = curves[nb_curves - 1];
                cur_curve->curvature2D(cur_curve->get_upper_bound() - 2 * GlobalState::get_dt(), cur_curvature, direction);
                curvatures(i) = cur_curvature;
            }
        }
    }

    controlPoints->set_curvatures(curvatures);
}

void Spline::get_control_curvatures(Eigen::VectorXd &curvatures)
{
    controlPoints->get_curvatures(curvatures);
}

void Spline::get_spline_points(Eigen::MatrixXd &line_points)
{
    int rows = 0;
    for (auto it = array_points.begin(); it != array_points.end(); it++)
    {
        rows += (*it).rows();
    }
    line_points = Eigen::MatrixXd(rows, 3);
    int current_row = 0;
    for (auto it = array_points.begin(); it != array_points.end(); it++)
    {
        const int new_block_rows = (*it).rows();
        line_points.block(current_row, 0, new_block_rows, 3) = *it;
        current_row += new_block_rows;
    }
}

void Spline::get_curvature_sign(Eigen::VectorXd &curvature_sign)
{
    const int nb_points = controlPoints->size();
    Eigen::MatrixXd points;
    controlPoints->get_points(points);
    curvature_sign = Eigen::VectorXd::Zero(nb_points);
    for (int i = 1; i < nb_points - 1; i++)
    {
        Eigen::Vector3d l1, l2;
        l1 = points.row(i) - points.row(i - 1);
        l2 = points.row(i + 1) - points.row(i);
        const double signed_angle = signed_angle2D(l1, l2);
        curvature_sign(i) = signed_angle < 0 ? -1 : 1;
    }
    if (cycle)
    {
        Eigen::Vector3d l0, lm, ln;
        l0 = points.row(1) - points.row(0);
        lm = points.row(0) - points.row(nb_points - 1);
        ln = points.row(nb_points - 1) - points.row(nb_points - 2);
        const double signed_angle_0 = signed_angle2D(lm, l0);
        const double signed_angle_n = signed_angle2D(ln, lm);
        curvature_sign(0) = signed_angle_0 < 0 ? -1 : 1;
        curvature_sign(nb_points - 1) = signed_angle_n < 0 ? -1 : 1;
    }
    else
    {
        curvature_sign(0) = curvature_sign(1);
        curvature_sign(nb_points - 1) = curvature_sign(nb_points - 2);
    }
}

// 1 means you are first duplicate, 2 means you are second duplicate.
void Spline::get_duplicates(Eigen::VectorXi &duplicates)
{
    const int nb_points = controlPoints->size();
    duplicates.setZero(nb_points);
    Eigen::MatrixXd points;
    controlPoints->get_points(points);
    for (int i = 0; i < nb_points; i++)
    {
        if ((points.row(pos_modulo(i, nb_points)) - points.row(pos_modulo(i + 1, nb_points))).norm() < EPSILON_APPROX)
        {
            duplicates(i) = 1;
            duplicates(i + 1) = 2;
        }
    }
    if (!cycle)
    {
        duplicates(nb_points - 1) = 1;
        duplicates(0) = 2;
    }
}

/*
    FINAL SOLUTION FUNCTIONS
*/

/*
    Compute the desired sign of the curvature: -1 if negative, +1 if positive, 0 if alignment
*/

/*
    Get the points which are not constraints
*/
void Spline::get_no_constraints(const Eigen::MatrixXd &points, Eigen::VectorXi &no_constraints)
{
    const double DUPLICATE_LIMIT = 1e-2;
    const int nb_points = points.rows();
    no_constraints = Eigen::VectorXi::Zero(nb_points);
    for (int i = 0; i < nb_points - 1; i++)
    {
        const Eigen::Vector3d p0 = points.row(i).transpose();
        const Eigen::Vector3d p1 = points.row(i + 1).transpose();
        if ((p0 - p1).norm() < DUPLICATE_LIMIT)
        {
            no_constraints(i) = 1;
            no_constraints(i + 1) = 1;
        }
    }
    if (!cycle)
    {
        no_constraints(0) = 1;
        no_constraints(nb_points - 1) = 1;
    }
}

const double ALIGNMENT_LIMIT = 1e-3;
void Spline::get_curvature_sign(const Eigen::MatrixXd &points, const Eigen::VectorXi &no_constraints, Eigen::VectorXi &curvature_sign)
{
    const int nb_points = points.rows();
    curvature_sign = Eigen::VectorXi::Zero(nb_points);
    for (int i = 0; i < nb_points; i++)
    {
        const Eigen::Vector3d p0 = points.row(pos_modulo(i - 1, nb_points)).transpose();
        const Eigen::Vector3d p1 = points.row(pos_modulo(i, nb_points)).transpose();
        const Eigen::Vector3d p2 = points.row(pos_modulo(i + 1, nb_points)).transpose();
        const Eigen::Vector3d l1 = (p1 - p0).normalized();
        const Eigen::Vector3d l2 = (p2 - p1).normalized();
        const double cross_product = (l1.cross(l2)).norm();
        if (cross_product < ALIGNMENT_LIMIT)
        {
            curvature_sign(i) = 0;
        }
        else
        {
            const double signed_angle = signed_angle2D(l1, l2);
            curvature_sign(i) = signed_angle < 0 ? -1 : 1;
        }
    }

    for (int i = 0; i < nb_points; i++)
    {
        if (no_constraints(i))
        {
            if (no_constraints(pos_modulo(i - 1, nb_points)) == 1)
            {
                curvature_sign(i) = curvature_sign(pos_modulo(i + 1, nb_points));
            }
            else if (no_constraints(pos_modulo(i + 1, nb_points)) == 1)
            {
                curvature_sign(i) = curvature_sign(pos_modulo(i - 1, nb_points));
            }
        }
    }
}

/*
    Getting the no-constraint index.
    Perform the choice depending on whether there is a no-constraint point after or before.
*/
void get_index_depending_on_no_constraints(const int &i, const int &nb_points, const Eigen::VectorXi &no_constraints, int &index_previous, int &index, int &index_next)
{
    index_previous = pos_modulo(i - 1, nb_points);
    index = pos_modulo(i, nb_points);
    index_next = pos_modulo(i + 1, nb_points);
    if (no_constraints(index))
    {
        if (no_constraints(index_previous))
        {
            index_previous = pos_modulo(i, nb_points);
            index = pos_modulo(i + 1, nb_points);
            index_next = pos_modulo(i + 2, nb_points);
        }
        else if (no_constraints(index_next))
        {
            index_previous = pos_modulo(i - 2, nb_points);
            index = pos_modulo(i - 1, nb_points);
            index_next = pos_modulo(i, nb_points);
        }
    }
    // std::cout << i << ": " << index_previous << "-" << index << "-" << index_next << std::endl;
}

/*
    Estimation of the tangent and curvature
*/
void Spline::estimate_curvature(const Eigen::MatrixXd &points, const Eigen::VectorXi &no_constraints, const Eigen::VectorXi &curvature_sign, Eigen::MatrixXd &tangents, Eigen::VectorXd &curvatures)
{
    const int nb_points = points.rows();
    tangents = Eigen::MatrixXd(nb_points, 3);
    curvatures = Eigen::VectorXd(nb_points);

    for (int i = 0; i < nb_points; i++)
    {
        int index, index_previous, index_next;
        get_index_depending_on_no_constraints(i, nb_points, no_constraints, index_previous, index, index_next);
        const Eigen::Vector3d p0 = points.row(index_previous).transpose();
        const Eigen::Vector3d p1 = points.row(index).transpose();
        const Eigen::Vector3d p2 = points.row(index_next).transpose();
        if (curvature_sign(i) == 0)
        {
            if (!no_constraints(i))
            {
                if (curvature_sign(pos_modulo(i - 1, nb_points)) == 0)
                {
                    tangents.row(i) = (p1 - p0).transpose().normalized();
                }
                else
                {
                    tangents.row(i) = (p2 - p1).transpose().normalized();
                }
            }
            else
            {
                if (no_constraints(pos_modulo(i - 1, nb_points)))
                {
                    tangents.row(i) = (p1 - p0).transpose().normalized();
                }
                else if (no_constraints(pos_modulo(i + 1, nb_points)))
                {
                    tangents.row(i) = (p2 - p1).transpose().normalized();
                }
            }
        }
        else
        {
            Interpolator *I;
            Eigen::Matrix3d points;
            Eigen::Vector3d tangent;
            points.row(0) = p0.transpose();
            points.row(1) = p1.transpose();
            points.row(2) = p2.transpose();
            get_Interpolator(points, &I);
            if (!no_constraints(i))
            {
                I->derivative(1, tangent);
            }
            else
            {
                if (no_constraints(pos_modulo(i - 1, nb_points)))
                {
                    I->derivative(I->get_lower_bound() + GlobalState::dt, tangent);
                }
                else if (no_constraints(pos_modulo(i + 1, nb_points)))
                {
                    I->derivative(I->get_upper_bound() - GlobalState::dt, tangent);
                }
            }
            tangents.row(i) = tangent.transpose();
        }
    }

    for (int i = 0; i < nb_points; i++)
    {
        if (curvature_sign(i) == 0)
        {
            curvatures(i) = 0.0;
        }
        else
        {
            int index, index_previous, index_next;
            get_index_depending_on_no_constraints(i, nb_points, no_constraints, index_previous, index, index_next);
            const Eigen::Vector3d p0 = points.row(index_previous).transpose();
            const Eigen::Vector3d p1 = points.row(index).transpose();
            const Eigen::Vector3d p2 = points.row(index_next).transpose();
            const Eigen::Vector3d t0 = tangents.row(index_previous).transpose();
            const Eigen::Vector3d t1 = tangents.row(index).transpose();
            const Eigen::Vector3d t2 = tangents.row(index_next).transpose();

            if (no_constraints(i))
            {
                Interpolator *I;
                Eigen::Matrix3d points;
                double curvature;
                points.row(0) = p0.transpose();
                points.row(1) = p1.transpose();
                points.row(2) = p2.transpose();
                get_Interpolator(points, &I);
                if (no_constraints(pos_modulo(i - 1, nb_points)))
                {
                    I->curvature2D(I->get_lower_bound() + 2 * GlobalState::dt, curvature);
                }
                else if (no_constraints(pos_modulo(i + 1, nb_points)))
                {
                    I->curvature2D(I->get_upper_bound() - 2 * GlobalState::dt, curvature);
                }
                curvatures(i) = curvature;
            }
            else
            {
                Interpolator *I;
                Eigen::Matrix3d points;
                double curvature_interpolator;
                points.row(0) = p0.transpose();
                points.row(1) = p1.transpose();
                points.row(2) = p2.transpose();
                get_Interpolator(points, &I);
                I->curvature2D(1.0 + 2 * GlobalState::dt, curvature_interpolator);
                if (use_g1_curvature)
                {
                    Clothoid_Curve G1_fitting_0_1(p0, t0, p1, t1);
                    Clothoid_Curve G1_fitting_1_2(p1, t1, p2, t2);
                    const double curvature_0 = G1_fitting_0_1.curvature_end();
                    const double curvature_1 = G1_fitting_1_2.curvature_beginning();
                    curvatures(i) = (curvature_0 + curvature_1) / 2; // G1-clothoid curvature mean. Should take the max?

                    // If the two curvture of the G1 fitting are opposite or if the result curvature is not of the same sign as the desired shape
                    // it is meaningless to make the average of both G1 fitting curvature
                    if ((curvature_0 * curvature_1) < 0 || (curvatures(i) * curvature_sign(i) < 0))
                    { // Bending curvature
                        curvatures(i) = curvature_interpolator;
                    }
                }
                else
                {
                    curvatures(i) = curvature_interpolator;
                }
            }
        }
    }
}

// Get the curvature at points.

void Spline::get_spline_curvature_points(Eigen::MatrixXd &curvature_points)
{
    curvature_points = Eigen::MatrixXd(parameter_array.size(), 3);
    double current_length = 0;
    int index_curve = 0;
    double current_lower_bound = curves[0]->get_lower_bound();
    for (int i = 0; i < parameter_array.size(); i++)
    {
        double effective_param = parameter_array[i] - current_lower_bound; // it's always parameter - curve_lowerbound
        if (effective_param > curves[pos_modulo(index_curve, curves.size())]->get_upper_bound() - curves[pos_modulo(index_curve, curves.size())]->get_lower_bound())
        {
            index_curve += 1;
            current_lower_bound += curves[index_curve - 1]->get_upper_bound() - curves[index_curve - 1]->get_lower_bound() + curves[pos_modulo(index_curve, curves.size())]->get_lower_bound();
            effective_param = parameter_array[i] - current_lower_bound;
        }

        double signed_curvature = 0;
        Eigen::Vector3d curvature_direction;
        bool too_low = effective_param + curves[pos_modulo(index_curve, curves.size())]->get_lower_bound() < curves[pos_modulo(index_curve, curves.size())]->get_lower_bound() + 2 * GlobalState::get_dt();
        bool too_high = effective_param + curves[pos_modulo(index_curve, curves.size())]->get_lower_bound() > curves[pos_modulo(index_curve, curves.size())]->get_upper_bound() - 2 * GlobalState::get_dt();
        if ((!too_low) && (!too_high))
        {
            curves[pos_modulo(index_curve, curves.size())]->curvature2D(effective_param + curves[pos_modulo(index_curve, curves.size())]->get_lower_bound(), signed_curvature, curvature_direction);
        }
        else
        {
            if (too_low)
            {
                curves[pos_modulo(index_curve, curves.size())]->curvature2D(curves[pos_modulo(index_curve, curves.size())]->get_lower_bound() + 2 * GlobalState::get_dt(), signed_curvature, curvature_direction);
            }
            else if (too_high)
            {
                curves[pos_modulo(index_curve, curves.size())]->curvature2D(curves[pos_modulo(index_curve, curves.size())]->get_upper_bound() - 2 * GlobalState::get_dt(), signed_curvature, curvature_direction);
            }
        }
        curvature_points.row(i) = curvature_direction.transpose() * abs(signed_curvature);
    }
}

double Spline::get_total_length()
{
    if (total_length < 0)
    {
        total_length = 0;
        for (auto it = curves.begin(); it != curves.end(); it++)
        {
            const Curve *cur_curve = *it;
            total_length += cur_curve->get_length();
        }
    }
    return total_length;
}

void Spline::eval(const double &t, Eigen::VectorXd &result, int &index_curve)
{
    M_Assert(0 <= t && t <= get_total_length(), "Value of the spline should be in [0, length]");
    double length = curves[0]->get_length();
    int index_current_curve = 0;
    while (length < t)
    {
        index_current_curve += 1;
        length += curves[index_current_curve]->get_length();
    }
    length -= curves[index_current_curve]->get_length();
    double parameter = t - length + curves[index_current_curve]->get_lower_bound();
    Eigen::Vector3d res;
    curves[index_current_curve]->eval(parameter, res);
    result = res;
    index_curve = index_current_curve;
}

void Spline::eval(const double &t, Eigen::VectorXd &result)
{
    int index = 0;
    eval(t, result, index);
}

bool Spline::get_constrained_tangent(const int &index, Eigen::Vector3d &tan)
{
    return controlPoints->get_constrained_tangent(index, tan);
}
bool Spline::get_constrained_curvature(const int &index, double &cur)
{
    return controlPoints->get_constrained_curvature(index, cur);
}

/*
    Auxiliary functions
*/
void Spline::get_control_points(Eigen::MatrixXd &c_points)
{
    controlPoints->get_points(c_points);
}

void Spline::get_control_point(int index, Eigen::RowVector3d &c_point)
{
    controlPoints->get_point(index, c_point);
}

void Spline::get_closest_control_points(const Eigen::RowVector3d &ref_point, int &id, double &distance)
{
    Eigen::MatrixXd c_points;
    controlPoints->get_points(c_points);
    M_Assert(c_points.rows() > 0, "To get the closest control points there should be some control points.");
    id = 0;
    distance = (ref_point - c_points.row(0)).norm();

    for (int i = 1; i < c_points.rows(); i++)
    {
        const double cur_dis = (ref_point - c_points.row(i)).norm();
        if (cur_dis < distance)
        {
            distance = cur_dis;
            id = i;
        }
    }
}