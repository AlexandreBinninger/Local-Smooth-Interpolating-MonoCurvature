#include "controller.h"

double Controller::epsilon_selection = 5e-3;
bool Controller::flag_add_point = false;
bool Controller::flag_select_point = false;
bool Controller::mouse_left_pressed = false;
bool Controller::mouse_right_pressed = false;
bool Controller::multi_selection = false;
bool Controller::rotation = false;
Eigen::Vector3f Controller::pos_down = Eigen::Vector3f::Zero();
Eigen::Vector3f Controller::pos_move = Eigen::Vector3f::Zero();
Eigen::Vector3f Controller::pos_up = Eigen::Vector3f::Zero();

std::set<int> Controller::selected_points = std::set<int>();

Eigen::Vector3d Controller::VEC_UP   ;
Eigen::Vector3d Controller::VEC_RIGHT;
Eigen::Vector3d Controller::VEC_DOWN ;
Eigen::Vector3d Controller::VEC_LEFT ;
double Controller::angle_rot = (2*M_PI)/120;
Eigen::MatrixXd Controller::ROT_TRIGO ;
Eigen::MatrixXd Controller::ROT_ANTETRIGO ;

std::vector<Eigen::Vector3d> Controller::lasso_points = std::vector<Eigen::Vector3d>();
bool Controller::constraints_just_curvature = true;


Eigen::Vector3d Controller::constraint_tangent = Eigen::Vector3d(1.0, 0., 0.);
double Controller::constraint_curvature = 1.;
void Controller::load_constraints(const Eigen::Vector3d & tan, const double& cur){
    constraint_tangent = tan;
    constraint_curvature = cur;
}
void Controller::load_constraints(int index){
    GlobalState::spline->get_constrained_tangent(index, constraint_tangent);
    GlobalState::spline->get_constrained_curvature(index, constraint_curvature);
}

double Controller::step_move = 5e-2;
unsigned char Controller::move_directions = 0;

Controller::Controller()
{
    Controller::VEC_UP << 0., 1., 0.;
    Controller::VEC_RIGHT << 1., 0., 0.;
    Controller::VEC_DOWN << 0., -1., 0.;
    Controller::VEC_LEFT << -1., 0., 0.;
    ROT_TRIGO = Eigen::MatrixXd(3, 3);
    ROT_ANTETRIGO = Eigen::MatrixXd(3, 3);
    rotationMatrix2D(angle_rot, ROT_TRIGO);
    rotationMatrix2D(-angle_rot, ROT_ANTETRIGO);
}



bool Controller::select_point(const Eigen::RowVector3d &coord, int& id)
{
    double distance;
    GlobalState::get_closest_control_points(coord, id, distance);
    if (distance * GlobalState::total_zoom() < epsilon_selection * GlobalState::get_point_size())
    {
        return true;
    }
    return false;
}

void Controller::clear_selected()
{
    selected_points.clear();
}

void Controller::move_point(const Eigen::Vector3d& point_in, const Eigen::Vector3d& center, const unsigned char& m_dir, Eigen::Vector3d& point_out){
    const double scale = ((double) 1.0/GlobalState::total_zoom()) * step_move;
    point_out = point_in;
    if (m_dir & MOVE_DIR::ROT_LEFT)  point_out = center + ROT_TRIGO * (point_out-center);
    if (m_dir & MOVE_DIR::ROT_RIGHT) point_out = center + ROT_ANTETRIGO * (point_out-center);
    if (m_dir & MOVE_DIR::UP)    point_out += VEC_UP*scale;
    if (m_dir & MOVE_DIR::RIGHT) point_out += VEC_RIGHT*scale;
    if (m_dir & MOVE_DIR::DOWN)  point_out += VEC_DOWN*scale;
    if (m_dir & MOVE_DIR::LEFT)  point_out += VEC_LEFT*scale;
}

void Controller::duplicate_selected(){
    if (selected_points.empty()){
        return;
    }
    int count =0;
    std::set<int> new_selected;
    for (auto it = selected_points.begin(); it!= selected_points.end(); it++){
        const int index = (*it) + count;
        new_selected.insert(index);
        count++;
        Eigen::RowVector3d point;
        GlobalState::get_control_point(index, point);
        GlobalState::spline->add_control_points(point, index);
    }
    selected_points = new_selected;
}

bool Controller::ismoving(){return !!move_directions;}