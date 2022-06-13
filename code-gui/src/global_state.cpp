#include "global.h"
#include "model/control_points.h"
#include "model/spline.h"
#include "controller/controller.h"
#include "view/view_points.h"
#include <igl/unproject_on_plane.h>
#include <igl/project.h>

int global_log_counter = 0;
bool GlobalState::to_be_updated = true;
std::unique_ptr<Eigen::Matrix3d> GlobalState::ROTATION_PI_2(new Eigen::Matrix3d());
bool GlobalState::to_input_constraints = false;

bool GlobalState::draw()
{

    // Get the control points and display them
    Eigen::MatrixXd control_points;
    // getter data
    get_control_points(control_points);
    if (control_points.rows() == 0){
        view->clear_screen();
        return false;
    }

    if (must_constraints_input()){
        std::set<int> selected_points = get_selected_points();
        if (selected_points.size() == 1){
            view->clear_screen();
            int index_selected = *selected_points.begin();
            view->display_constraints(control_points.row(index_selected).transpose(), GlobalState::controller->constraint_tangent.transpose(), GlobalState::controller->constraint_curvature);
        }
        return false;
    }
    // display points
    if (view->show_curvature_color){
        // Get the curvature points and display their color
        Eigen::VectorXd curvature_control_points;
        spline->get_control_curvatures(curvature_control_points);
        view->display_control_points_curvaturecolors(control_points, get_selected_points(), curvature_control_points);
    } else{
        view->display_control_points(control_points, get_selected_points());
    }

    view->display_polygon(control_points, GlobalState::spline->cycle);

    // Get the line points and display them
    Eigen::MatrixXd line_points;
    // getter data
    get_spline_points(line_points);
    // display points
    view->display_spline(line_points);
    // Get the tangent points and display them
    Eigen::MatrixXd tangent_control_points;
    if (view->flag_display_tangents_control_points){
        spline->get_control_tangents(tangent_control_points);
        view->display_tangents(control_points, tangent_control_points);
    }

    if (view->flag_display_osculating_circle_control_points){
        if (!view->flag_display_tangents_control_points){spline->get_control_tangents(tangent_control_points);}
        Eigen::VectorXd curvature_control_points;
        spline->get_control_curvatures(curvature_control_points);
        view->display_osculating_circles(control_points, tangent_control_points, curvature_control_points);
    }

    if (view->flag_display_curvature)
    {
        // Get the curvature points and display them
        Eigen::MatrixXd curvature_points;
        // getter data
        spline->get_spline_curvature_points(curvature_points);
        // display points
        view->display_curvature(line_points, -curvature_points*view->scale_curvature);
    }

    // See with controller if we should deactivate the update
    if (controller->ismoving())
    {
        activate_update();
        controller->move_selected_points();
    }

    return false;
}

// Interface for control points
void GlobalState::add_control_points(const Eigen::MatrixXd &m_points)
{
    spline->add_control_points(m_points);
}

void GlobalState::get_control_point(int index, Eigen::RowVector3d &points)
{
    spline->get_control_point(index, points);
}

void GlobalState::get_control_points(Eigen::MatrixXd &points)
{
    spline->get_control_points(points);
}

void GlobalState::remove_control_point(int index)
{
    spline->remove_control_point(index);
}

void GlobalState::move_control_point(const Eigen::Vector3d &m_points, int index)
{
    spline->move_control_point(m_points, index);
}

void GlobalState::get_closest_control_points(const Eigen::RowVector3d &ref_point, int &id, double &distance)
{
    spline->get_closest_control_points(ref_point, id, distance);
}

void GlobalState::get_spline_points(Eigen::MatrixXd &points)
{
    spline->get_spline_points(points);
}

std::set<int> GlobalState::get_selected_points()
{
    return controller->get_selected_points();
}

void GlobalState::set_uni_control_points_colors(const Eigen::RowVector3f &color)
{
    view->set_uni_control_points_colors(color);
}
void GlobalState::set_multi_control_points_colors(const Eigen::MatrixXf &colors)
{
    view->set_multi_control_points_colors(colors);
}
void GlobalState::set_point_size(const int &point_size)
{
    view->set_point_size(point_size);
}

int GlobalState::get_point_size()
{
    return view->get_point_size();
}

void GlobalState::set_uni_spline_colors(const Eigen::RowVector4f &color)
{
    view->set_uni_spline_colors(color);
}
void GlobalState::set_multi_spline_colors(const Eigen::MatrixXf &colors)
{
    view->set_multi_spline_colors(colors);
}
void GlobalState::set_line_width(const int &line_width)
{
    view->set_line_width(line_width);
}

void GlobalState::update_lasso()
{
    view->display_lasso(controller->lasso_points);
}

void GlobalState::get_xy_coordinates(const int &mouse_x, const int &mouse_y, Eigen::MatrixXf &XYZ_Plane_Coords)
{
    Eigen::Vector2f UV(mouse_x, viewer->core().viewport[3] - mouse_y);
    Eigen::Vector4f Plane(0, 0, 1, 0);
    Eigen::MatrixXf coords_on_plane;
    igl::unproject_on_plane(UV, viewer->core().proj, viewer->core().viewport, Plane, coords_on_plane);
    XYZ_Plane_Coords = ((coords_on_plane) / (viewer->core().camera_base_zoom * viewer->core().camera_zoom) - viewer->core().camera_translation - viewer->core().camera_base_translation).transpose();
}

double GlobalState::get_diagonal_length()
{
    Eigen::MatrixXf coord_bottom_left, coord_top_right;
    get_xy_coordinates(0, 0, coord_bottom_left);
    get_xy_coordinates(viewer->core().viewport[2], viewer->core().viewport[3], coord_top_right);
    const double scale = (coord_top_right - coord_bottom_left).norm();
    if (scale < EPSILON_APPROX)
        GlobalState::activate_update();
    return scale;
}

void GlobalState::recompute_spline_points()
{ 
    spline->recompute_spline();
}