#include "circle.h"

bool are_aligned(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2){
    const Eigen::Vector2d v1 = (p1-p0).normalized();
    const Eigen::Vector2d v2 = (p2-p1).normalized();
    const double cross = v1(1)*v2(0) - v1(0)*v2(1);
    return (abs(cross) < EPSILON_APPROX);
}

bool are_aligned(const Eigen::Matrix3d& points){
    const Eigen::MatrixXd points_2d = points.block(0, 0, 3, 2);
    return are_aligned(points_2d.row(0).transpose(), points_2d.row(1).transpose(), points_2d.row(2).transpose());
}

void circle_radius_center(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, double &radius, Eigen::Vector2d& center){
    // Based in the equation 2x c_x + 2y c_y + (r^2-c_x^2 - c_y^2) = x^2 + y^2 fulfilled by the three points
    if (are_aligned(p0, p1, p2)){
        throw std::runtime_error("Points are aligned");
    }
    const std::vector<Eigen::Vector2d> points_vector{p0, p1, p2};
    Eigen::Matrix3d A = Eigen::Matrix3d::Ones(3, 3);
    Eigen::Vector3d b(3);
    for (int i=0; i<3; i++){
        const double xi = points_vector[i](0);
        const double yi = points_vector[i](1);
        A(i, 0) = 2*xi;
        A(i, 1) = 2*yi;
        b(i) = xi*xi + yi*yi;
    }
    Eigen::Vector3d c = A.colPivHouseholderQr().solve(b);

    center << c(0), c(1);
    radius = sqrt(c(2) + center.squaredNorm());
}


void circle_radius_center(const Eigen::Matrix3d& points, double &radius, Eigen::Vector3d& center){
    const Eigen::MatrixXd points_2d = points.block(0, 0, 3, 2);
    Eigen::Vector2d center_2d;
    circle_radius_center(points_2d.row(0).transpose(), points_2d.row(1).transpose(), points_2d.row(2).transpose(), radius, center_2d);
    center = Eigen::Vector3d::Zero();
    center << center_2d, 0;
}