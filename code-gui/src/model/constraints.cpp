#include "constraints.h"

Constraints::Constraints(){
    is_constrained = Eigen::MatrixXi::Zero(0, 2);
    tangents = Eigen::MatrixXd::Zero(0, 3);
    curvatures = Eigen::VectorXd::Zero(0);
}
Constraints::Constraints(const int nb_points){
    is_constrained = Eigen::MatrixXi::Zero(nb_points, 2);
    tangents = Eigen::MatrixXd::Zero(nb_points, 3);
    curvatures = Eigen::VectorXd::Zero(nb_points);
}


void Constraints::reset(const int nb_points){
    is_constrained = Eigen::MatrixXi::Zero(nb_points, 2);
    tangents = Eigen::MatrixXd::Zero(nb_points, 3);
    curvatures = Eigen::VectorXd::Zero(nb_points);
}

void Constraints::reset(const Eigen::MatrixXi& m_is_constrained, const Eigen::MatrixXd& m_tangents, const Eigen::VectorXd& m_curvatures){
    is_constrained = m_is_constrained;
    tangents = m_tangents;
    curvatures = m_curvatures;
}

void Constraints::dump(Eigen::MatrixXi& m_is_constrained, Eigen::MatrixXd& m_tangents, Eigen::VectorXd& m_curvatures){
    m_is_constrained = is_constrained;
    m_tangents = tangents;
    m_curvatures = curvatures;
}


void Constraints::add_control_tangent(const int index, const Eigen::Vector3d& tangent){
    M_Assert(index < is_constrained.rows(), "Index should be less than the number of points.");
    is_constrained(index, 0) = 1;
    tangents.row(index) = tangent.transpose();
}

void Constraints::add_control_curvature(const int index, const double& curvature){
    M_Assert(index < is_constrained.rows(), "Index should be less than the number of points.");
    is_constrained(index, 1) = 1;
    curvatures(index) = curvature;
}

void Constraints::remove_control_tangent(const int index){
    M_Assert(index < is_constrained.rows(), "Index should be less than the number of points.");
    is_constrained(index, 0) = 0;
}

void Constraints::remove_control_curvature(const int index){
    M_Assert(index < is_constrained.rows(), "Index should be less than the number of points.");
    is_constrained(index, 1) = 0;
}

void Constraints::get_curvature_constraints_index(Eigen::VectorXi& const_index){
    const_index = is_constrained.col(1);
}
void Constraints::get_constraints(const int& index, Eigen::Vector3d& tan, double& cur){
    tan = tangents.row(index).transpose();
    cur = curvatures(index);
}

bool Constraints::get_tangent_constraints(const int& index, Eigen::Vector3d& tan){
    tan = tangents.row(index).transpose();
    return is_constrained(index, 0);
}

bool Constraints::get_curvature_constraints(const int& index, double& cur){
    cur = curvatures(index);
    return is_constrained(index, 1);
}


void Constraints::apply_constraints(Eigen::MatrixXd& ref_tangents, Eigen::VectorXd& ref_curvatures){
    for (int i=0; i<is_constrained.rows(); i++){
        if (is_constrained(i, 0) == 1){
            ref_tangents.row(i) = tangents.row(i);
        }
        if (is_constrained(i, 1) == 1){
            ref_curvatures(i) = curvatures(i);
        }
    }
}

void Constraints::check(){
    M_Log(is_constrained, "");
    M_Log(tangents, "tangents\n");
    M_Log(curvatures, "curvatures\n");
}
