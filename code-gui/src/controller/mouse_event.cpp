#include "mouse_event.h"

void Controller::selector(const Eigen::MatrixXd &coords)
{
    int id_selected;
    const bool is_selected = select_point(coords.row(0), id_selected);
    if (is_selected)
    {
        auto it_selected = selected_points.find(id_selected);
        if (it_selected == selected_points.end())
        {
            if (multi_selection)
                selected_points.insert(id_selected);
            else{
                selected_points.clear();
                selected_points.insert(id_selected);
            }
        }
        else
        {
            selected_points.erase(it_selected);
        }
        GlobalState::activate_update();
    }
}


void Controller::add_point(const Eigen::MatrixXd& coords){
    if (selected_points.empty()){
        GlobalState::add_control_points(coords);
        selected_points.insert(GlobalState::spline->size_control_points()-1);
    } else{
        const int index_selected = *(selected_points.rbegin());
        GlobalState::spline->add_control_points(coords, index_selected+1);
        if (!multi_selection){
            selected_points.clear();
        }
        selected_points.insert(index_selected+1);
    }
}

void Controller::move_selected_point(int mouse_x, int mouse_y)
{
    auto it = selected_points.begin();

    Eigen::MatrixXf new_point;
    GlobalState::get_xy_coordinates(mouse_x, mouse_y, new_point);
    const Eigen::MatrixXd new_point_double = new_point.cast<double>();
    GlobalState::move_control_point(new_point_double.transpose(), *it);
    GlobalState::activate_update();
}

void Controller::end_lasso(){
    Eigen::MatrixXd control_points;
    GlobalState::get_control_points(control_points);
    for (int i=0; i<control_points.rows(); i++){
        Eigen::Vector3d point = control_points.row(i).transpose();
        if (inside_polygon(point, lasso_points)){
            if (!multi_selection){
                selected_points.clear();
            }
            selected_points.insert(i);
        }
    }
    lasso_points.clear();
    GlobalState::update_lasso();
    GlobalState::activate_update();
}

bool Controller::callback_mouse_down(Viewer &viewer, int button, int modifier)
{
    Eigen::MatrixXf XYZ_Plane_Coords;
    GlobalState::get_xy_coordinates(viewer.current_mouse_x, viewer.current_mouse_y, XYZ_Plane_Coords);
    pos_down = XYZ_Plane_Coords.transpose();
    if (button == (int)Viewer::MouseButton::Left)
    {
        flag_select_point = !!(modifier & GLFW_MOD_SHIFT);
        mouse_left_pressed = true;
        return true;
    }
    else if (button == (int)Viewer::MouseButton::Right)
    {
        mouse_right_pressed = true;
        return false;
    }
    return true;
}
bool Controller::callback_mouse_move(Viewer &viewer, int mouse_x, int mouse_y)
{
    Eigen::MatrixXf XYZ_Plane_Coords;
    GlobalState::get_xy_coordinates(mouse_x, mouse_y, XYZ_Plane_Coords);
    pos_move = XYZ_Plane_Coords.transpose();
    if (mouse_left_pressed)
    {
        if (!flag_select_point && selected_points.size() == 1)
        {
            if (GlobalState::must_constraints_input()){
                int index_selected = *selected_points.begin();
                const Eigen::Vector3d center_osculating = pos_move.transpose().cast<double>();
                Eigen::RowVector3d point_index;
                GlobalState::spline->get_control_point(index_selected, point_index); 
                double sign_curvature = sign(constraint_curvature);
                Eigen::Vector3d point_to_centeroscu = center_osculating-point_index.transpose();
                double new_curvature = sign_curvature * 1.0/(point_to_centeroscu).norm();
                constraint_curvature = new_curvature;
                if (!constraints_just_curvature){
                    constraint_tangent = -sign_curvature * (*GlobalState::ROTATION_PI_2 * point_to_centeroscu.normalized());
                } 
                GlobalState::activate_update();
            } else{
                move_selected_point(mouse_x, mouse_y);
            }
        } else if (flag_select_point){
            lasso_points.push_back(pos_move.cast<double>());
            GlobalState::update_lasso();
        }
    }
    return false;
}
bool Controller::callback_mouse_up(Viewer &viewer, int button, int modifier)
{

    Eigen::MatrixXf XYZ_Plane_Coords;
    GlobalState::get_xy_coordinates(viewer.current_mouse_x, viewer.current_mouse_y, XYZ_Plane_Coords);
    pos_up = XYZ_Plane_Coords.transpose();
    if (button == (int)Viewer::MouseButton::Left)
    {
        end_lasso();
        mouse_left_pressed = false;
        if ((pos_up-pos_down).norm() * GlobalState::total_zoom() < epsilon_selection) // if we did not move too much
        {
            const Eigen::MatrixXd XYZ_Plane_Coords_double = pos_up.transpose().cast<double>();
            if (!flag_select_point && !(GlobalState::must_constraints_input()))
            {
                add_point(XYZ_Plane_Coords_double);
                GlobalState::activate_update();
            }
            else if (flag_select_point)
            {
                selector(XYZ_Plane_Coords_double);
            }
            return true;
        }
    }
    else if (button == (int)Viewer::MouseButton::Right)
    {
        mouse_right_pressed = false;
        return false;
    }
    return true;
}

bool Controller::callback_mouse_scroll(Viewer &viewer, float delta_y)
{
    GlobalState::activate_update();
    return false;
}

void init_mouse_events()
{
    Viewer &viewer = GlobalState::getViewer();
    viewer.callback_mouse_down = GlobalState::controller->callback_mouse_down;
    viewer.callback_mouse_up = GlobalState::controller->callback_mouse_up;
    viewer.callback_mouse_move = GlobalState::controller->callback_mouse_move;
    viewer.callback_mouse_scroll = GlobalState::controller->callback_mouse_scroll;
}