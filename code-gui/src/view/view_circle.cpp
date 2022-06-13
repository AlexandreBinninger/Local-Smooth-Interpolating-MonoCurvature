#include "view_circle.h"

void View::display_circle(Viewer &viewer, const int &id_viewer, const Eigen::VectorXd &center, const Eigen::VectorXd &p0, const Eigen::MatrixXf &color, const double &width, const int resolution)
{
    Eigen::MatrixXd P1 = Eigen::MatrixXd::Zero(resolution, 3), P2 = Eigen::MatrixXd::Zero(resolution, 3);

    Eigen::VectorXd v = p0 - center;
    for (int i = 0; i < resolution; i++)
    {
        Eigen::MatrixXd rot(3, 3);
        rotationMatrix2D(M_PI*2 * i/resolution, rot);
        const Eigen::VectorXd new_point = center + rot * v;
        P1.row(i) = new_point;
        P2.row((resolution + (i-1))%resolution) = new_point;
    }
    add_lines(*GlobalState::viewer, id_viewer, P1, P2, color, width);
}