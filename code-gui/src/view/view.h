#pragma once
#include "../global.h"
#include "../model/control_points.h"

class View{
    public: // parameters

    // Parameters for control points
    Eigen::MatrixXf control_points_colors = Eigen::RowVector3f(220., 20., 60.)/255;
    Eigen::MatrixXf control_points_colors_negative_curvature = Eigen::RowVector3f(20., 220., 60.)/255;
    Eigen::RowVector3f selected_control_points_colors = Eigen::RowVector3f(30., 235., 195.)/255;
    int pointSize = 50; 
    bool show_curvature_color = true;

    Eigen::RowVector3f polygon_color = Eigen::RowVector3f(196., 196., 196.)/255;
    bool display_control_polygon = false;

    // Parameters for spline
    Eigen::MatrixXf spline_colors = Eigen::RowVector4f(0., 0., 0., 1.);
    int lineWidth = 10;
    double ratio_line = 10000; //ratio of the linewidth and the scale. Based on experiments.
    int cylinder_resolution = 4;
    int sphere_side = 20;
    bool display_splines = true;

    // Parameters for lasso
    Eigen::RowVector3f lasso_color = Eigen::RowVector3f(127., 127., 127.)/255;

    // Parameters for tangents
    bool flag_display_tangents_control_points = false;
    double length_tangents = 1.0;
    Eigen::RowVector4d tangents_color_control_points = Eigen::RowVector4d(255., 102., 0., 255.)/255.0;

    // Parameter for osculating circle
    bool flag_display_osculating_circle_control_points = false;
    Eigen::RowVector4f osculating_color_control_points = 0.7 * Eigen::RowVector4f(127., 255., 17./0.7, 255.)/255.0;
    int resolution_osculating = 30;

    // Parameters for curvature
    bool flag_display_curvature = true;
    int point_to_curvature = 5;
    double scale_curvature = 1.0; //comb scale: scale the curvature representation
    bool use_height_curvature_color = false; // use the height to determine the color of the curvature on the graph 
    Eigen::MatrixXf curvature_colors = Eigen::RowVector4f(238., 130., 238., 255.)/(255); // else use this

    View(){}

    // Parameter modifiers
    void set_uni_control_points_colors(const Eigen::RowVector3f& color);
    void set_multi_control_points_colors(const Eigen::MatrixXf& colors);
    void set_point_size(const int& point_size);
    int get_point_size(){return pointSize;}
    double get_ratio_line(){return ratio_line;}
    double get_linewidth_world(const int &lineWidth){return lineWidth * GlobalState::get_diagonal_length()/ratio_line;}

    void set_uni_spline_colors(const Eigen::RowVector4f& color);
    void set_multi_spline_colors(const Eigen::MatrixXf& colors);
    void set_line_width(const int& line_width);

    // Display Functions
    void set_curve(Viewer &viewer, const int &id_viewer, const Eigen::MatrixXd &points, const Eigen::MatrixXf &color, const int &lineWidth = 2);
    void display_circle(Viewer &viewer, const int &id_viewer, const Eigen::VectorXd &center, const Eigen::VectorXd &p0, const Eigen::MatrixXf &color, const double &width, const int resolution);
    void display_control_points(const Eigen::MatrixXd& points);
    void display_control_points(const Eigen::MatrixXd& points, const std::set<int>& selected_points);
    void display_control_points_curvaturecolors(const Eigen::MatrixXd &points, const std::set<int> &selected_points, const Eigen::VectorXd &curvatures);
    void clear_control_points();
    void display_spline(const Eigen::MatrixXd& points);
    void display_polygon(const Eigen::MatrixXd& control_points, const bool & cycle = false);
    void display_tangents(const Eigen::MatrixXd& points, const Eigen::MatrixXd& tangents);
    void display_osculating_circles(const Eigen::MatrixXd& points, const Eigen::MatrixXd& tangents, const Eigen::VectorXd& curvatures);
    void display_curvature(const Eigen::MatrixXd& line_points, const Eigen::MatrixXd& curvature_points);
    void display_lasso(const std::vector<Eigen::Vector3d> &lasso_points);
    void display_constraints(const Eigen::Vector3d& control_point, const Eigen::Vector3d& constraint_tangent, const double & constraint_curv);

    // Clear functions
    void clear_screen();
};


template <typename T>
void format_color(const Eigen::Matrix<T, -1, -1>& color_in, const int& n, Eigen::Matrix<T, -1, -1>& color_out){
    color_out = Eigen::Matrix<T, -1, -1>(n, color_in.cols());
    const int nb_col_entry = color_in.rows();
    for (int i=0; i<n; i++){
        color_out.row(i) = color_in.row(i%nb_col_entry);
    }
}