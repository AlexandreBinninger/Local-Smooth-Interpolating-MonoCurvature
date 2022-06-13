#pragma once

// C/C++ standard libraries
#include <iostream>
#include <stdlib.h>
#include <assert.h> /* assert */
#include <memory>
#include <stdexcept>
#include <deque>

// File with all the global variables shared among several parts of the program

// Simple logging system
#define ALLOW_LOG
#define EXCEPTION_DUMP

#ifdef ALLOW_LOG
#define M_Log(Expr, Msg)                     \
    __M_Log(#Expr, __FILE__, __LINE__, Msg); \
    std::cout << Expr << std::endl;
#else
#define M_Log(Expr, Msg) ;
#endif

#define M_Debug() \
    M_Log("", "Debug")

#define ALLOW_COUNT

extern int global_log_counter;
#ifdef ALLOW_COUNT
#define M_Count()                     \
    M_Log(global_log_counter, "Count:"); global_log_counter++;
#else
#define M_Count(Expr, Msg) ;
#endif
#define M_Reset_Count() (global_log_counter=0);


// Assertion system. Source: https://stackoverflow.com/a/37264642

#ifndef NDEBUG
#define M_Assert(Expr, Msg) \
    __M_Assert(#Expr, Expr, __FILE__, __LINE__, Msg)
#else
#define M_Assert(Expr, Msg) ;
#endif

void __M_Assert(const char *expr_str, bool expr, const char *file, int line, const char *msg);
void __M_Log(const char *expr_str, const char *file, int line, const char *Msg);

// Viewer
#include <igl/opengl/glfw/Viewer.h>
using Viewer = igl::opengl::glfw::Viewer;

#define EPSILON_APPROX 1e-6

// Forward Class Definitions
class Spline;
class Controller;
class View;


// Singleton pattern: inspired from https://gameprogrammingpatterns.com/singleton.html

class GlobalState
{
private:
    GlobalState() {}

public:
    
    static GlobalState &instance()
    {
        static GlobalState *instance = new GlobalState();
        return *instance;
    }

    ~GlobalState() {}

//Some Variables

    static bool to_be_updated;
    constexpr static double dt = 1e-3;
    static bool to_input_constraints;
    
    static std::unique_ptr<Spline> spline;
    static std::unique_ptr<Viewer> viewer;
    static int id_plane, id_control_points, id_spline, id_tangent, id_curvature, id_osculating, id_lasso, id_control_polygon, id_constraint_line, id_constraint_circle; // ids of the viewer.data()

    static std::unique_ptr<Controller> controller;
    static std::unique_ptr<View> view;

    static std::unique_ptr<Eigen::Matrix3d> ROTATION_PI_2;
    static Eigen::MatrixXd Input_Points;


// Getters

    // The viewer can be accessed from everywhere in this manner.
    static Viewer& getViewer(){
        return *viewer;
    }

    static inline double get_dt(){
        return dt;
    }

    // Update
    static bool must_be_updated(){return to_be_updated;}
    static void activate_update(){to_be_updated = true;}
    static void deactivate_update(){to_be_updated = false;}
    bool draw();
    static inline float total_zoom(){return viewer->core().camera_base_zoom * viewer->core().camera_zoom;};

    // Interface for control points
    static void add_control_points(const Eigen::MatrixXd& m_points);
    static void get_control_point(int index, Eigen::RowVector3d& points);
    static void get_control_points(Eigen::MatrixXd& points);
    static void remove_control_point(int index);
    static void move_control_point(const Eigen::Vector3d &m_points, int index);

    // Constraints inputs
    static bool must_constraints_input(){return to_input_constraints;}
    static void activate_constraints_input(){to_input_constraints = true; viewer->data(id_constraint_line).show_lines = true; viewer->data(id_constraint_circle).show_lines = true; viewer->data(id_constraint_line).set_visible(true); viewer->data(id_constraint_circle).set_visible(true); }
    static void deactivate_constraints_input(){to_input_constraints = false; viewer->data(id_constraint_line).show_lines = false; viewer->data(id_constraint_circle).show_lines = false; viewer->data(id_constraint_line).set_visible(false); viewer->data(id_constraint_circle).set_visible(false); }
    static void modify_constraints_bool(){
        if (must_constraints_input()){
            deactivate_constraints_input();
            M_Log("Deactivate update constraints", "I");
        }
        else{
            activate_constraints_input();
            M_Log("Activate update constraints", "I");
        }
    }

    // Interface for spline points
    static void get_spline_points(Eigen::MatrixXd& points);
    static void get_closest_control_points(const Eigen::RowVector3d& ref_points, int& id, double& distance);
    static void recompute_spline_points();

    // Interface to access some controller parameters
    static std::set<int> get_selected_points();

    // Interface for View modifier
    static void set_uni_control_points_colors(const Eigen::RowVector3f& color);
    static void set_multi_control_points_colors(const Eigen::MatrixXf& colors);
    static void set_point_size(const int& point_size);
    static int get_point_size();

    static void set_uni_spline_colors(const Eigen::RowVector4f& color);
    static void set_multi_spline_colors(const Eigen::MatrixXf& colors);
    static void set_line_width(const int& line_width);
    static void update_lasso();


    static void get_xy_coordinates(const int& mouse_x, const int& mouse_y, Eigen::MatrixXf& XYZ_Plane_Coords);
    static double get_diagonal_length();
};