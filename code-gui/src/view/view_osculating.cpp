#include "view_osculating.h"


void View::display_osculating_circles(const Eigen::MatrixXd& points, const Eigen::MatrixXd& tangents, const Eigen::VectorXd& curvatures){
    M_Assert(points.rows() == curvatures.rows(), "same number of points and curvatures");
    M_Assert(tangents.rows() == curvatures.rows(), "same number of tangents and curvatures");
    const int id_viewer = GlobalState::id_osculating;
    GlobalState::getViewer().data(id_viewer).clear();
    Eigen::MatrixXd rot(3, 3);
    rotationMatrix2D(M_PI_2, rot);


    const Eigen::MatrixXd normals = (rot * tangents.transpose()).transpose();
    for (int i=0; i<points.rows(); i++){
        Eigen::VectorXd p0 = points.row(i).transpose();
        Eigen::VectorXd center;
        if (abs(curvatures(i)) > 1e-5)
            center = p0 + 1/curvatures(i) * normals.row(i).transpose();
        else
            center = p0;
        display_circle(GlobalState::getViewer(), id_viewer, center, p0, osculating_color_control_points, get_linewidth_world(lineWidth/3), resolution_osculating);
    }
}