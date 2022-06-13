#pragma once

#include "../global.h"
#include "../model/spline.h"
#include "../utils/polygons.h"


class Controller{
    private:
    static std::set<int> selected_points;
    static double epsilon_selection; // the maximum distance from which we can select the point

    public:
    static bool flag_add_point;
    static bool flag_select_point ;
    static bool mouse_left_pressed;
    static bool mouse_right_pressed;
    static bool multi_selection;
    static bool rotation;

    static Eigen::Vector3f pos_down;
    static Eigen::Vector3f pos_move;
    static Eigen::Vector3f pos_up;
    

    static Eigen::Vector3d VEC_UP;   //  {0., 1., 0.}
    static Eigen::Vector3d VEC_RIGHT;//  {1., 0., 0.}
    static Eigen::Vector3d VEC_DOWN; //  {0., -1., 0.}
    static Eigen::Vector3d VEC_LEFT; //  {-1., 0., 0.}
    static double angle_rot;
    static Eigen::MatrixXd ROT_TRIGO;
    static Eigen::MatrixXd ROT_ANTETRIGO;

    static std::vector<Eigen::Vector3d> lasso_points;

    // Movement
    static double step_move;
    static unsigned char move_directions;

    // Constraints
    static bool constraints_just_curvature;

    // current constraints
    static Eigen::Vector3d constraint_tangent;
    static double constraint_curvature;
    static void load_constraints(const Eigen::Vector3d & tan, const double& cur);
    static void load_constraints(int index);

    enum MOVE_DIR{
        UP = 1,
        RIGHT = 2,
        DOWN = 4,
        LEFT = 8,
        ROT_LEFT = 16,
        ROT_RIGHT = 32
    };

    Controller();


    static bool select_point(const Eigen::RowVector3d& coord, int& id); // get the closest point via the model. Return true if something has been selected, false otherwise
    static void selector(const Eigen::MatrixXd& coords);
    static void add_point(const Eigen::MatrixXd& coords);
    static void clear_selected();
    static std::set<int> get_selected_points(){return selected_points;}

    // Mouse
    static bool callback_mouse_down(Viewer &viewer, int button, int modifier);
    static bool callback_mouse_move(Viewer &viewer, int mouse_x, int mouse_y);
    static bool callback_mouse_up(Viewer &viewer, int button, int modifier);
    static bool callback_mouse_scroll(Viewer &viewer, float delta_y);


    // Keyboard
    static bool key_down(Viewer &viewer, unsigned char key, int modifiers);
    static bool key_pressed(Viewer &viewer, unsigned char key, int modifiers);
    static bool key_up(Viewer &viewer, unsigned char key, int modifiers);

    // Utils
    static void move_point(const Eigen::Vector3d& point_in, const Eigen::Vector3d& center, const unsigned char& m_dir, Eigen::Vector3d& point_out);
    static void move_selected_points();
    static void move_selected_point(int mouse_x, int mouse_y);
    static void remove_selected_points();
    static void duplicate_selected();
    static bool ismoving();
    static void end_lasso();
};

void modification();