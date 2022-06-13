 #pragma once

#include "../global.h"
#include "constraints.h"

class ControlPoints{
    private:
    Eigen::MatrixXd points; //n*3 matrix
    Eigen::MatrixXd tangents; // n*3 matrix
    Eigen::VectorXd curvatures; // n vector

    public:
    std::unique_ptr<Constraints> constraints;
    ControlPoints();
    ControlPoints(const Eigen::MatrixXd& m_points);

    ControlPoints(const Eigen::MatrixXd& m_points, const Eigen::MatrixXd& m_tangents, const Eigen::VectorXd& m_curvatures);

    void set_points(const Eigen::MatrixXd& points);

    void set_tangents(const Eigen::MatrixXd& m_tangents);

    void set_curvatures(const Eigen::MatrixXd& m_curvatures);

    void add_points(const Eigen::MatrixXd& points, int index =-1); // can be a nx2/nx3 matrix, or a 2d/3d vector


    void remove_point(int index);
    void move_point(const Eigen::VectorXd &m_points, int index);

    void clear();


    void get_points(Eigen::MatrixXd& m_points);
    void get_point(const int& index, Eigen::RowVector3d &point);


    void get_tangents(Eigen::MatrixXd&  m_tangents);
    void get_curvatures(Eigen::VectorXd&  m_curvatures);


    // getter for constraints
    bool get_constrained_tangent(const int& index, Eigen::Vector3d& tan);
    bool get_constrained_curvature(const int& index, double& cur);

    bool empty(){return points.rows()==0;}
    int size(){return points.rows();}
};