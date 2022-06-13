#include "init_model.h"
std::unique_ptr<Spline> GlobalState::spline(new Spline());


void set_plane(const double& scale, const Eigen::VectorXd center){
  // Inline mesh of a plane
  Eigen::MatrixXd V = (Eigen::MatrixXd(4, 3) << -1.0, -1.0, 0.0,
                             -1.0, 1.0, 0.0,
                             1.0, 1.0, 0.0,
                             1.0, -1.0, 0.0)
                                .finished() * scale;
  V.block(0, 0, 4, 1) = V.block(0, 0, 4, 1) + Eigen::Vector4d::Ones()*center(0);
  V.block(0, 1, 4, 1) = V.block(0, 1, 4, 1) + Eigen::Vector4d::Ones()*center(1); 
  const Eigen::MatrixXi F = (Eigen::MatrixXi(2, 3) << 1, 3, 2,
                             1, 4, 3)
                                .finished()
                                .array() -
                            1;
  GlobalState::viewer->data(GlobalState::id_plane).clear();
  GlobalState::viewer->data(GlobalState::id_plane).set_mesh(V, F);
}

void set_plane_control_points(){
    Eigen::MatrixXd c_p;
    GlobalState::get_control_points(c_p);
    const Eigen::Vector3d center = c_p.colwise().mean();
    const Eigen::MatrixXd centered_c_p = c_p.rowwise() - center.transpose();
    double scale = 2*centered_c_p.cwiseAbs().maxCoeff();
    if (scale < EPSILON_APPROX) scale = Standard_Models::scale_model;
    set_plane(scale, center);
}

void initialize_model()
{
    Eigen::MatrixXd Points;
    Standard_Models::input_points(Points);

    GlobalState::instance().add_control_points(Points);
    Eigen::VectorXd center = Points.colwise().mean();
    set_plane_control_points();
    GlobalState::activate_update();
    return;
}
