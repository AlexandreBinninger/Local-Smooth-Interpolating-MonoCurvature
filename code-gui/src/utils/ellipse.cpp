#include "ellipse.h"

void ellipse_center(const Eigen::Vector2d &_p0, const Eigen::Vector2d &_p1, const Eigen::Vector2d &_p2, Eigen::Vector2d &center, Eigen::Vector2d &ax1, Eigen::Vector2d &ax2, double &max_theta, bool &reverse)
{
    // Based in the equation 2x c_x + 2y c_y + (r^2-c_x^2 - c_y^2) = x^2 + y^2 fulfilled by the three points
    if (are_aligned(_p0, _p1, _p2))
    {
        throw std::runtime_error("Points are aligned");
    }
    Eigen::Vector2d p0 = _p0, p1 = _p1, p2 = _p2;
    Eigen::Vector2d p01 = p0 - p1;
    Eigen::Vector2d p21 = p2 - p1;
    if (p01.squaredNorm() < p21.squaredNorm())
    {
        Eigen::Vector2d tmp;
        // swap 1
        tmp = p0;
        p0 = p2;
        p2 = tmp;
        // swap 2
        tmp = p01;
        p01 = p21;
        p21 = tmp;
        reverse = true;
    }
    else
    {
        reverse = false;
    }


    const Eigen::Vector2d c = (p0 + p1) / 2; // center of the circle on which the center of the ellipse can be
    const Eigen::Vector2d u = p0 - c;

    // get the orientation
    const double cross = p01(0) * p21(1) - p01(1) * p21(0);
    const double sign_cross = (cross > 0) ? 1 : -1;
    const double dot = p01.dot(p21);
    const double sign_dot = (dot > 0) ? 1 : -1;

    //direction of theta must be given by the direction of the vector p0-p1, p0-p2, i.e. by the cross product
    double theta = sign_cross * (M_PI_2);
    //range of theta is given by the dot product. |theta| < pi/2 if the dot product is positive and |theta| > pi/2 otherwise.
    double incr = sign_cross * (M_PI_2)*0.5;
    const int nb_iter = 16;
    Eigen::Vector2d c_theta, _ax1, _ax2, _ax3;
    double cos_alfa=0, sin_alfa=0;
    Eigen::MatrixXd rot_theta = Eigen::Matrix2d::Zero();
    for (int i = 0; i < nb_iter; i++)
    {
        rotationMatrix2D(theta, rot_theta);
        c_theta = c + rot_theta * u;
        _ax1 = p0 - c_theta;
        _ax2 = p1 - c_theta;
        _ax3 = p2 - c_theta;

        const double dp13 = _ax3.dot(_ax1);
        const double dp23 = _ax3.dot(_ax2);

        const double n1 = _ax1.dot(_ax1);
        const double n2 = _ax2.dot(_ax2);

        cos_alfa = (dp13 / n1);
        sin_alfa = (dp23 / n2);

        double ellipse_equation = cos_alfa * cos_alfa + sin_alfa * sin_alfa;
        if (sign_dot < 0)
        {
            theta += ((ellipse_equation > 1) && (sin_alfa > 0)) ? incr : -incr;
        }
        else
        {
            theta += ((ellipse_equation > 1) && (cos_alfa < 0)) ? incr : -incr;
        }
        incr *= 0.5;
    }
    ax1 = _ax1;
    ax2 = _ax2;
    center = c_theta;
    max_theta = atan2(sin_alfa, cos_alfa);
    if (max_theta < 0)
    {
        // can happen because of imprecision
        max_theta += 2 * M_PI;
    }
}

void ellipse_center(const Eigen::Matrix3d &points, Eigen::Vector3d &center, Eigen::Vector3d &ax1, Eigen::Vector3d &ax2, double &max_theta, bool &reverse)
{
    const Eigen::MatrixXd points_2d = points.block(0, 0, 3, 2);
    Eigen::Vector2d center_2d, ax1_2d, ax2_2d;
    ellipse_center(points_2d.row(0).transpose(), points_2d.row(1).transpose(), points_2d.row(2).transpose(), center_2d, ax1_2d, ax2_2d, max_theta, reverse);
    center = Eigen::Vector3d::Zero();
    ax1 = Eigen::Vector3d::Zero();
    ax2 = Eigen::Vector3d::Zero();
    center << center_2d, 0;
    ax1 << ax1_2d, 0;
    ax2 << ax2_2d, 0;
}