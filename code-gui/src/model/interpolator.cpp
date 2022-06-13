#include "interpolator.h"

/*
    ---------------
        Line
    ---------------
    Linear Interpolator. Mainly here for test purposes.
*/

void Line::eval(const double &t, Eigen::Vector3d &result) const
{
    M_Assert(bounds.first <= t && t <= bounds.second, "Value inside the bounds [0., 2.]");
    if (t < 1)
    {
        result = (points.row(0) * (1 - t) + points.row(1) * t);
    }
    else
    {
        result = (points.row(1) * (2 - t) + points.row(2) * (t - 1));
    }
    result = result.transpose();
}


/*
    ---------------
        Circle
    ---------------
    Circular Interpolator. Fit the only circle passing through 3 points;
*/

void Circle::initialize_circle(){
    circle_radius_center(points, radius, center);
    ax1 = points.row(0).transpose() - center;
    ax2 = Eigen::Vector3d::Zero();
    ax2(0) = -ax1(1);
    ax2(1) = ax1(0);
    if (!is_right_direction()) ax2 = -ax2;
    theta1 = get_angle(points.row(1).transpose() - center);
    theta2 = get_angle(points.row(2).transpose() - center);
}

void Circle::eval(const double &t, Eigen::Vector3d &result) const
{
    M_Assert(bounds.first <= t && t <= bounds.second, "Value inside the bounds [0., 2.]");
    if (t < 1)
    {
        const double shifted_t = theta1 * t;
        result = center + ax1 * cos(shifted_t) + ax2 * sin(shifted_t);
    }
    else
    {
        const double shifted_t = theta1 + (theta2-theta1) * (t-1);
        result = center + ax1 * cos(shifted_t) + ax2 * sin(shifted_t);
    }
}

inline double Circle::get_angle(const Eigen::Vector3d& v) const{
    const double r2_cos_theta = v.dot(ax1);
    const double r2_sin_theta = v.dot(ax2);
    double theta = atan2(r2_sin_theta, r2_cos_theta);
    theta += (theta<0) ? 2*M_PI : 0;
    return theta;
}

bool Circle::is_right_direction() const
{
    const Eigen::Vector3d v1 = points.row(1).transpose() - center;
    const Eigen::Vector3d v2 = points.row(2).transpose() - center;
    const double theta1 = get_angle(v1);
    const double theta2 = get_angle(v2);
    return theta1 <= theta2;
}

void Circle::full_shape(Eigen::MatrixXd& display_points, const int& nb_points) const
{
    display_points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i=0; i<nb_points; i++){
        const double angle = 2*M_PI * (double) i /(nb_points-1);
        display_points.row(i) << (center + ax1 * cos(angle) + ax2 * sin(angle)).transpose();
    }
}


void Circle::full_shape_separated(std::vector<Eigen::MatrixXd>& display_points, const int& nb_points) const
{
    display_points = std::vector<Eigen::MatrixXd>();

    Eigen::MatrixXd current_points(nb_points, 3);
    for (int i=0; i<nb_points; i++){
        const double angle = theta1 * (double) i /(nb_points-1);
        current_points.row(i) << (center + ax1 * cos(angle) + ax2 * sin(angle)).transpose();
    }
    display_points.push_back(current_points);

    current_points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i=0; i<nb_points; i++){
        const double angle = ((theta2-theta1) * (double) i /(nb_points-1)) + theta1;
        current_points.row(i) << (center + ax1 * cos(angle) + ax2 * sin(angle)).transpose();
    }
    display_points.push_back(current_points);
    
    current_points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i=0; i<nb_points; i++){
        const double angle = ((2*M_PI-theta2) * (double) i /(nb_points-1)) + theta2;
        current_points.row(i) << (center + ax1 * cos(angle) + ax2 * sin(angle)).transpose();
    }
    display_points.push_back(current_points);
}




/*
    ---------------
        Ellipse
    ---------------
    Elliptic Interpolator. Fit the ellipse passing through 3 points as describe by Yuksel. Two points are on the principal axes.
*/

void Ellipse::initialize_ellipse(){

    ellipse_center(points, center, ax1, ax2, max_theta, reverse);
}

void Ellipse::eval(const double &t, Eigen::Vector3d &result) const
{
    M_Assert(bounds.first <= t && t <= bounds.second, "Value inside the bounds [0., 2.]");

    const double reparam_t = reverse ? (2-t) : t;
    if (reparam_t < 1)
    {
        const double shifted_t = reparam_t*M_PI_2;
        result = center + ax1 * cos(shifted_t) + ax2 * sin(reparam_t*M_PI_2);
    }
    else
    {
        const double shifted_t = M_PI_2 + (max_theta-M_PI_2) * (reparam_t-1);
        result = center + ax1 * cos(shifted_t) + ax2 * sin(shifted_t);
    }
}


void Ellipse::full_shape(Eigen::MatrixXd& display_points, const int& nb_points) const
{
    display_points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i=0; i<nb_points; i++){
        const double angle = 2*M_PI * (double) i /(nb_points-1);
        display_points.row(i) << (center + ax1 * cos(angle) + ax2 * sin(angle)).transpose();
    }
}


void Ellipse::full_shape_separated(std::vector<Eigen::MatrixXd>& display_points, const int& nb_points) const
{
    display_points = std::vector<Eigen::MatrixXd>();

    Eigen::MatrixXd current_points(nb_points, 3);
    for (int i=0; i<nb_points; i++){
        const double angle = M_PI_2 * (double) i /(nb_points-1);
        current_points.row(i) << (center + ax1 * cos(angle) + ax2 * sin(angle)).transpose();
    }
    display_points.push_back(current_points);

    current_points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i=0; i<nb_points; i++){
        const double angle = ((max_theta-M_PI_2) * (double) i /(nb_points-1)) + M_PI_2;
        current_points.row(i) << (center + ax1 * cos(angle) + ax2 * sin(angle)).transpose();
    }
    display_points.push_back(current_points);
    
    current_points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i=0; i<nb_points; i++){
        const double angle = ((2*M_PI-max_theta) * (double) i /(nb_points-1)) + max_theta;
        current_points.row(i) << (center + ax1 * cos(angle) + ax2 * sin(angle)).transpose();
    }
    display_points.push_back(current_points);
}



/*
    ---------------
        Hybrid
    ---------------
    Hybrid Interpolator. Fit ellipse or circle depending on conditions.
*/

void Hybrid::initialize_hybrid(){
    Circle *circ = new Circle(points);
    double theta_1, theta_2;
    circ->get_theta(theta_1, theta_2);
    if ((theta_1 < M_PI_2) && (theta_2 < M_PI_2 + theta_1)){
        interp = circ;
    } else{
        interp = new Ellipse(points);
    }
}

void Hybrid::eval(const double &t, Eigen::Vector3d &result) const {
    interp->eval(t, result);
}

