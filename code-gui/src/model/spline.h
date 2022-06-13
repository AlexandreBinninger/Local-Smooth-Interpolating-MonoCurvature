#pragma once
#include "../global.h"

#include "../utils/misc.h"

#include "../view/view_curve.h"
#include "control_points.h"
#include "curve.h"
#include "interpolator.h"
#include "clothoid_line.h"
#include "clothoid_3arcs.h"
#include "clothoid_curve.h"

enum Spline_Type{
    Curve_3Arcs_Clothoid,
    Curve_Line_Clothoid,
};

class Spline{
    // A spline is defined as an object which holds the points defined by the control points in the Global State.
    
    private:
    double total_length = -1;
    // Data 
    std::deque<Eigen::MatrixXd> array_points; //deque of n*3 matrix
    std::deque<Curve*> curves; // These curves are such that curves[i] is the curves between point i and point i+1. If cycling is true, then curve[n] is the curve between last and first points
    std::deque<double> parameter_array;

    // Internal functions
    void compute_3arcs_clothoid(const Eigen::MatrixXd &ordered_points, std::deque<Curve *> &new_curves);
    void compute_clothoid_line(const Eigen::MatrixXd &ordered_points, std::deque<Curve *> &new_curves);
    void compute_curves(const Eigen::MatrixXd &ordered_points, std::deque<Curve*>& new_curves);
    void initialize_control_points(const Eigen::MatrixXd &m_points);
    void recompute_curves();

    void compute_spline(const std::vector<int>& indices);
    void compute_control_tangents();
    void compute_control_curvatures();

    void get_base_curvatures(const Eigen::MatrixXd &ordered_points, const bool & cycle, Eigen::MatrixXd & tangents, Eigen::VectorXd & curvatures);


    public:
    std::unique_ptr<ControlPoints> controlPoints;

    Spline_Type spline_type = Curve_Line_Clothoid;
    Curvature_Change_Type curvature_change_type = CHANGE_TANGENT;  
    Clothoid_Kind clothoid_kind = TANGENT;


    // Data parameters
    bool cycle = true; // is the final curve cycling?
    int nb_points_per_curve = 100; // the number of points per curve

    // Options on Clothoid-Line 
    bool use_g1_curvature = true;
    bool use_correction_CLC = true;
    bool refine = true;


    // Intersection parameters
    bool intersection_detection = false;
    int neighbors_intersection = 5;
    
    Spline();

    // Get inteprolator with the spline variables 
    void get_Interpolator(const Eigen::Matrix3d& m_points, Interpolator **I);

    // Parameter change

    // Reset of the spline
    void softreset(); // Reset curves and spline points, not the control point.
    void hardreset(); // Reset of everything

    // Control Points Management
    void add_control_points(const Eigen::MatrixXd& m_points, int index=-1);
    void set_control_points(const Eigen::MatrixXd& m_points);
    void get_control_points(Eigen::MatrixXd& points);
    void get_control_point(int index, Eigen::RowVector3d& c_point);
    void remove_control_point(int index);
    void move_control_point(const Eigen::Vector3d &m_points, int index);

    void get_control_tangents(Eigen::MatrixXd &tangents);
    void get_control_curvatures(Eigen::VectorXd &curvatures);
    
    // number of control points
    int size_control_points();
    void get_closest_control_points(const Eigen::RowVector3d& ref_point, int& id, double& distance);

    // Spline points
    void get_spline_points(Eigen::MatrixXd& line_points);
    void recompute_spline(); 

    // Curvature points
    void get_curvature_sign(Eigen::VectorXd& curvature_sign);
    void get_duplicates(Eigen::VectorXi &duplicates);
    void get_spline_curvature_points(Eigen::MatrixXd& curvature_points);

    // Final methods functions
    void get_no_constraints(const Eigen::MatrixXd &points, Eigen::VectorXi &no_constraints);
    void get_curvature_sign(const Eigen::MatrixXd &points, const Eigen::VectorXi &no_constraints, Eigen::VectorXi &curvature_sign);
    void estimate_curvature(const Eigen::MatrixXd &points, const Eigen::VectorXi &no_constraints, const Eigen::VectorXi &curvature_sign, Eigen::MatrixXd &tangents, Eigen::VectorXd &curvatures);

    // useful for global parameterization
    double get_total_length();
    void eval(const double &t, Eigen::VectorXd &result, int& index_curve);
    void eval(const double &t, Eigen::VectorXd &result);

    // getter for constraints
    bool get_constrained_tangent(const int& index, Eigen::Vector3d& tan);
    bool get_constrained_curvature(const int& index, double& cur);
};