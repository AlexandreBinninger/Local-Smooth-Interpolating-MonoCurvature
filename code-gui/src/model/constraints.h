#pragma once
#include "../global.h"


class Constraints{
    private:
    Eigen::MatrixXi is_constrained; //n*2 matrix. First column is for tangent, second columns is for curvature
    Eigen::MatrixXd tangents; // n*3 matrix
    Eigen::VectorXd curvatures; // n vector

    public:
    Constraints();
    Constraints(const int nb_points);


    void reset(const int nb_points);
    void reset(const Eigen::MatrixXi& m_is_constrained, const Eigen::MatrixXd& m_tangents, const Eigen::VectorXd& m_curvatures);
    void dump(Eigen::MatrixXi& m_is_constrained, Eigen::MatrixXd& m_tangents, Eigen::VectorXd& m_curvatures);

    void add_control_tangent(const int index, const Eigen::Vector3d& tangent);
    void add_control_curvature(const int index, const double& curvature);
    void remove_control_tangent(const int index);
    void remove_control_curvature(const int index);
    
    void get_curvature_constraints_index(Eigen::VectorXi& const_index);
    void get_constraints(const int& index, Eigen::Vector3d& tan, double& cur); 
    bool get_tangent_constraints(const int& index, Eigen::Vector3d& tan); 
    bool get_curvature_constraints(const int& index, double& cur);

    void apply_constraints(Eigen::MatrixXd& tangents, Eigen::VectorXd& curvatures);
    void check();
};