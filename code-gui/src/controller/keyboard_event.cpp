#include "keyboard_event.h"

void Controller::move_selected_points()
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (auto it = selected_points.begin(); it != selected_points.end(); it++)
    {
        Eigen::RowVector3d old_point;
        GlobalState::get_control_point(*it, old_point);
        center += old_point.transpose();
    }
    center = center / selected_points.size();
    for (auto it = selected_points.begin(); it != selected_points.end(); it++)
    {
        Eigen::RowVector3d old_point;
        Eigen::Vector3d new_point;
        GlobalState::get_control_point(*it, old_point);
        Controller::move_point(old_point, center, move_directions, new_point);
        GlobalState::move_control_point(new_point, *it);
    }
    GlobalState::activate_update();
}

void Controller::remove_selected_points()
{
    for (auto it = selected_points.rbegin(); it != selected_points.rend(); it++)
    {
        GlobalState::remove_control_point(*it);
    }
    if (selected_points.size() == 1)
    {
        const int index = *(selected_points.begin()) - 1;
        selected_points.clear();
        selected_points.insert(index < 0 ? 0 : index);
    }
    else
        selected_points.clear();
    GlobalState::activate_update();
}

bool Controller::key_down(Viewer &viewer, unsigned char key, int modifiers)
{
    switch (key)
    {
    case 'C':
        GlobalState::view->flag_display_curvature = !GlobalState::view->flag_display_curvature;
        viewer.data(GlobalState::id_curvature).set_visible(GlobalState::view->flag_display_curvature);
        GlobalState::activate_update();
        break;
    case 'T':
        if ((modifiers & GLFW_MOD_SHIFT)) // due to the fact that shift and 'T' have the same key. I have to check with the modifier.
        {
            return true;
        }
        GlobalState::view->flag_display_tangents_control_points = !GlobalState::view->flag_display_tangents_control_points;
        viewer.data(GlobalState::id_tangent).set_visible(GlobalState::view->flag_display_tangents_control_points);
        GlobalState::activate_update();
        break;
    case 'O':
        GlobalState::view->flag_display_osculating_circle_control_points = !GlobalState::view->flag_display_osculating_circle_control_points;
        viewer.data(GlobalState::id_osculating).set_visible(GlobalState::view->flag_display_osculating_circle_control_points);
        GlobalState::activate_update();
        break;
    case 'I':
        if (get_selected_points().size() == 1)
        {
            int index = *(get_selected_points().begin());
            load_constraints(index);
            GlobalState::modify_constraints_bool();
        }
        else
        {
            GlobalState::deactivate_constraints_input();
        }
        GlobalState::activate_update();
        break;
    case 'V':
        if (GlobalState::must_constraints_input())
        {
            const int index_selected = *(GlobalState::controller->get_selected_points().begin());
            GlobalState::spline->controlPoints->constraints->add_control_curvature(index_selected, GlobalState::controller->constraint_curvature);
            if (!GlobalState::controller->constraints_just_curvature)
            {
                GlobalState::spline->controlPoints->constraints->add_control_tangent(index_selected, GlobalState::controller->constraint_tangent);
            }
            GlobalState::deactivate_constraints_input();
            modification();
        }
        break;
    case 'D':
        remove_selected_points();
        break;
    case 'M':
        Controller::multi_selection = !Controller::multi_selection;
        break;
    case 'R':
        Controller::rotation = !Controller::rotation;
        break;
    case 'S':
        selected_points.clear();
        GlobalState::activate_update();
        break;
    case (GLFW_KEY_UP % 256):
        if (Controller::rotation)
            Controller::move_directions = Controller::move_directions | MOVE_DIR::ROT_LEFT;
        else
            Controller::move_directions = Controller::move_directions | MOVE_DIR::UP;
        if (GlobalState::must_constraints_input())
        {
            double sign_curvature = sign(constraint_curvature);
            constraint_curvature += sign_curvature * constraint_curvature / 10.0;
        }
        GlobalState::activate_update();
        break;
    case (GLFW_KEY_RIGHT % 256):
        if (Controller::rotation)
            Controller::move_directions = Controller::move_directions | MOVE_DIR::ROT_RIGHT;
        else
            Controller::move_directions = Controller::move_directions | MOVE_DIR::RIGHT;
        GlobalState::activate_update();
        break;
    case (GLFW_KEY_DOWN % 256):
        if (Controller::rotation)
            Controller::move_directions = Controller::move_directions | MOVE_DIR::ROT_RIGHT;
        else
            Controller::move_directions = Controller::move_directions | MOVE_DIR::DOWN;
        if (GlobalState::must_constraints_input())
        {
            double sign_curvature = sign(constraint_curvature);
            constraint_curvature -= sign_curvature * constraint_curvature / 10.0;
        }
        GlobalState::activate_update();
        break;
    case (GLFW_KEY_LEFT % 256):
        if (Controller::rotation)
            Controller::move_directions = Controller::move_directions | MOVE_DIR::ROT_LEFT;
        else
            Controller::move_directions = Controller::move_directions | MOVE_DIR::LEFT;
        GlobalState::activate_update();
        break;
    default:
        break;
    }
    return true;
}

bool Controller::key_pressed(Viewer &viewer, unsigned char key, int modifiers)
{
    switch (key)
    {
    case 'u':
        break;
    default:
        break;
    }
    return true;
}

bool Controller::key_up(Viewer &viewer, unsigned char key, int modifiers)
{
    switch (key)
    {
    case (GLFW_KEY_UP % 256):
        if (Controller::rotation)
            Controller::move_directions = Controller::move_directions & ~(MOVE_DIR::ROT_LEFT);
        else
            Controller::move_directions = Controller::move_directions & ~(MOVE_DIR::UP);
        break;
    case (GLFW_KEY_RIGHT % 256):
        if (Controller::rotation)
            Controller::move_directions = Controller::move_directions & ~(MOVE_DIR::ROT_RIGHT);
        else
            Controller::move_directions = Controller::move_directions & ~(MOVE_DIR::RIGHT);
        break;
    case (GLFW_KEY_DOWN % 256):
        if (Controller::rotation)
            Controller::move_directions = Controller::move_directions & ~(MOVE_DIR::ROT_RIGHT);
        else
            Controller::move_directions = Controller::move_directions & ~(MOVE_DIR::DOWN);
        break;
    case (GLFW_KEY_LEFT % 256):
        if (Controller::rotation)
            Controller::move_directions = Controller::move_directions & ~(MOVE_DIR::ROT_LEFT);
        else
            Controller::move_directions = Controller::move_directions & ~(MOVE_DIR::LEFT);
        break;

    default:
        break;
    }
    return true;
}

void init_keyboard_events()
{
    Viewer &viewer = GlobalState::getViewer();

    viewer.callback_key_down = GlobalState::controller->key_down;
    viewer.callback_key_pressed = GlobalState::controller->key_pressed;
    viewer.callback_key_up = GlobalState::controller->key_up;
}