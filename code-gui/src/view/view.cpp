#include "view.h"

// Parameters change

void View::set_uni_control_points_colors(const Eigen::RowVector3f &color)
{
    control_points_colors = color;
}
void View::set_multi_control_points_colors(const Eigen::MatrixXf &colors)
{
    control_points_colors = colors;
}
void View::set_point_size(const int &point_size)
{
    pointSize = point_size;
}

void View::set_uni_spline_colors(const Eigen::RowVector4f &color)
{
    spline_colors = color;
}
void View::set_multi_spline_colors(const Eigen::MatrixXf &colors)
{
    spline_colors = colors;
}
void View::set_line_width(const int &line_width)
{
    lineWidth = line_width;
}

void View::clear_screen()
{
    GlobalState::viewer->data(GlobalState::id_control_points).clear();
    GlobalState::viewer->data(GlobalState::id_spline).clear();
    GlobalState::viewer->data(GlobalState::id_tangent).clear();
    GlobalState::viewer->data(GlobalState::id_curvature).clear();
    GlobalState::viewer->data(GlobalState::id_osculating).clear();
    GlobalState::viewer->data(GlobalState::id_lasso).clear();
    GlobalState::viewer->data(GlobalState::id_control_polygon).clear();
}

void View::display_constraints(const Eigen::Vector3d &control_point, const Eigen::Vector3d &constraint_tangent, const double &constraint_curv)
{
    display_control_points(control_point.transpose());
    display_tangents(control_point.transpose(), constraint_tangent.transpose());
    Eigen::VectorXd constraint_curvature(1);
    constraint_curvature << constraint_curv;
    display_osculating_circles(control_point.transpose(), constraint_tangent.transpose(), constraint_curvature);

    GlobalState::getViewer().data(GlobalState::id_constraint_circle).clear();
    const Eigen::VectorXd normal = (*GlobalState::ROTATION_PI_2) * constraint_tangent;
    Eigen::VectorXd center;

    if (abs(constraint_curv) > 1e-5)
        center = control_point + 1 / constraint_curv * normal;
    else
        center = control_point;
    display_circle(GlobalState::getViewer(), GlobalState::id_constraint_circle, center, control_point, osculating_color_control_points, get_linewidth_world(lineWidth / 3), resolution_osculating);

    Eigen::MatrixXd line_poly(2, 3);
    line_poly.row(0) = control_point.transpose();
    line_poly.row(1) = control_point.transpose() + (1.0 / constraint_curv) * ((*GlobalState::ROTATION_PI_2) * constraint_tangent).transpose();
    set_curve(GlobalState::getViewer(), GlobalState::id_constraint_line, line_poly, polygon_color, lineWidth / (2.));
}