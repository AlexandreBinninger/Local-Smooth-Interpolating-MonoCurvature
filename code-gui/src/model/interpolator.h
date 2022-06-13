#pragma once

#include "../global.h"
#include "curve.h"

// Use of CRTP: https://stackoverflow.com/a/37887952/15493116
class Interpolator : public Curve
{
protected:
    /* Ordered set of points. We must have
    eval(0) = points.row(0).transpose();
    eval(1) = points.row(1).transpose();
    eval(2) = points.row(2).transpose();
    */
    Eigen::Matrix3d points;

    /* additional data to be added here by inheriting classes */

public:
    virtual ~Interpolator(){};
    virtual Interpolator *clone() const = 0;

    Interpolator()
    {
        points = Eigen::Matrix3d::Zero();
        bounds = std::make_pair<double, double>(0., 2.);
    }

    Interpolator(const Eigen::Matrix3d &m_points)
    {
        points = m_points;
        bounds = std::make_pair<double, double>(0., 2.);
    }

    Interpolator(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
        points = Eigen::Matrix3d::Zero();
        points.row(0) = p0.transpose();
        points.row(1) = p1.transpose();
        points.row(2) = p2.transpose();
        bounds = std::make_pair<double, double>(0., 2.);
    }

    Eigen::Matrix3d get_points(){
        return points;
    }

};

/*
    ---------------
        Line
    ---------------
    Linear Interpolator.
*/

class Line : public Interpolator
{
public:
    Line(){};
    Line(const Eigen::Matrix3d &m_points) : Interpolator(m_points){};
    Line(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) : Interpolator(p0, p1, p2){};
    ~Line() {}
    Interpolator *clone() const override { return new Line(points); }

    void eval(const double &t, Eigen::Vector3d &result) const override;
};


/*
    ---------------
        Circle
    ---------------
    Circular Interpolator. Fit the only circle passing through 3 points;
*/
#include "../utils/circle.h"

class Circle : public Interpolator
{
protected:
    Eigen::Vector3d center;
    double radius;

    Eigen::Vector3d ax1;
    Eigen::Vector3d ax2;
    double theta1;
    double theta2;

    void initialize_circle();
    inline double get_angle(const Eigen::Vector3d& p) const;

public:
    Circle(){M_Log("Calling default Circle constructor.", "Potential error");initialize_circle();};
    Circle(const Eigen::Matrix3d &m_points) : Interpolator(m_points){initialize_circle();};
    Circle(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) : Interpolator(p0, p1, p2){initialize_circle();};
    ~Circle() {}
    Interpolator *clone() const override { return new Circle(points); }

    bool is_right_direction() const; // tells whether ax2 is or not in the right direction.

    const void get_theta(double &theta_1, double& theta_2){theta_1 = theta1; theta_2 = theta2;};

    void eval(const double &t, Eigen::Vector3d &result) const override;

    void full_shape(Eigen::MatrixXd& display_points, const int& nb_points=50) const; // display full circle
    void full_shape_separated(std::vector<Eigen::MatrixXd>& display_points, const int& nb_points=50) const; // display full circle
};



/*
    ---------------
        Ellipse
    ---------------
    Elliptic Interpolator. Fit the ellipse passing through 3 points as describe by Yuksel. Two points are on the principal axes.
*/

#include "../utils/ellipse.h"

class Ellipse : public Interpolator
{
protected:
    Eigen::Vector3d center;

    Eigen::Vector3d ax1;
    Eigen::Vector3d ax2;
    bool reverse;
    double max_theta;

    void initialize_ellipse();

public:
    Ellipse(){M_Log("Calling default Ellipse constructor.", "Potential error");initialize_ellipse();};
    Ellipse(const Eigen::Matrix3d &m_points) : Interpolator(m_points){initialize_ellipse();};
    Ellipse(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) : Interpolator(p0, p1, p2){initialize_ellipse();};
    ~Ellipse() {}
    Interpolator *clone() const override { return new Ellipse(points); }


    void eval(const double &t, Eigen::Vector3d &result) const override;

    void full_shape(Eigen::MatrixXd& display_points, const int& nb_points=50) const; // display full ellipse
    void full_shape_separated(std::vector<Eigen::MatrixXd>& display_points, const int& nb_points=50) const; // display full ellipse
};



/*
    ---------------
        Hybrid
    ---------------
    Hybrid Interpolator. Fit ellipse or circle depending on conditions.
*/

#include "../utils/ellipse.h"

class Hybrid : public Interpolator
{
protected:

    Interpolator * interp;

    void initialize_hybrid();

public:
    Hybrid(){M_Log("Calling default Hybrid constructor.", "Potential error");initialize_hybrid();};
    Hybrid(const Eigen::Matrix3d &m_points) : Interpolator(m_points){initialize_hybrid();};
    Hybrid(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) : Interpolator(p0, p1, p2){initialize_hybrid();};
    ~Hybrid() {}
    Interpolator *clone() const override { return new Hybrid(points); }


    void eval(const double &t, Eigen::Vector3d &result) const override;

    void full_shape(Eigen::MatrixXd& display_points, const int& nb_points=50) const; // display full Hybrid
    void full_shape_separated(std::vector<Eigen::MatrixXd>& display_points, const int& nb_points=50) const; // display full Hybrid
};
