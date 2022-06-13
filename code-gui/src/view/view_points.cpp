#include "view_points.h"

void display_points(Viewer &viewer, const int &id_viewer, const Eigen::MatrixXd &points, const Eigen::MatrixXf &color, const int &pointSize)
{
    // Points must be a matrix nx2 or nx3. If col dimension is 2, the points are printed on the plane z=0.
    const int n_rows = points.rows(), n_cols = points.cols();
    if (n_rows == 0)
        return;
    M_Assert((n_cols == 2) || (n_cols == 3), "Points should be in R^2 or R^3. Point Matrix must therefore be nx2 or nx3");

    viewer.data(id_viewer).clear();
    const double scale = GlobalState::get_diagonal_length();
    const double sphere_radius = (pointSize * scale) / (GlobalState::view->get_ratio_line());
    Eigen::MatrixXd V, colors, C;
    Eigen::MatrixXi F;
    format_color<double>(color.cast<double>(), points.rows(), colors);
    hedra::point_spheres(points, sphere_radius, colors, GlobalState::view->sphere_side + 1, V, F, C);
    viewer.data(id_viewer).set_mesh(V, F);
    viewer.data(id_viewer).set_colors(C);
}

void View::clear_control_points()
{
    GlobalState::getViewer().data(GlobalState::id_control_points).clear();
}

void View::display_control_points(const Eigen::MatrixXd &points)
{
    clear_control_points();

    Eigen::MatrixXf cur_colors = control_points_colors;
    if (control_points_colors.rows() > 1 && control_points_colors.rows() != points.rows())
    {
        format_color<float>(control_points_colors, points.rows(), cur_colors);
    }
    display_points(GlobalState::getViewer(), GlobalState::id_control_points, points, cur_colors, pointSize);
}

void View::display_control_points(const Eigen::MatrixXd &points, const std::set<int> &selected_points)
{
    clear_control_points();
    Eigen::MatrixXf cur_colors;
    format_color<float>(control_points_colors, points.rows(), cur_colors);
    for (auto it = selected_points.begin(); it != selected_points.end(); it++)
    {
        cur_colors.row(*it) = selected_control_points_colors;
    }
    display_points(GlobalState::getViewer(), GlobalState::id_control_points, points, cur_colors, pointSize);
}


void View::display_control_points_curvaturecolors(const Eigen::MatrixXd &points, const std::set<int> &selected_points, const Eigen::VectorXd &curvatures)
{
    clear_control_points();
    Eigen::MatrixXf cur_colors;
    format_color<float>(control_points_colors, points.rows(), cur_colors);
    for (int i=0; i<curvatures.rows(); i++){
        if (curvatures(i) < 0){
            cur_colors.row(i) = control_points_colors_negative_curvature;
        }
    }
    for (auto it = selected_points.begin(); it != selected_points.end(); it++)
    {
        cur_colors.row(*it) = selected_control_points_colors;
    }
    display_points(GlobalState::getViewer(), GlobalState::id_control_points, points, cur_colors, pointSize);
}