#pragma once

#include "../global.h"
#include <utility>
#include "../utils/rotation.h"

class Curve
{
protected:
    std::pair<double, double> bounds;

public:
    virtual ~Curve(){};
    /* Evaluation is specific to each curve: it has to be defined for each implementation */
    virtual void eval(const double &t, Eigen::Vector3d &result) const = 0;
    Eigen::Vector3d operator()(const double &t){Eigen::Vector3d result; eval(t, result); return result;}

    double inline get_lower_bound() const {return bounds.first;}
    double inline get_upper_bound() const {return bounds.second;}
    double inline get_length() const {return bounds.second-bounds.first;}

    virtual void linspace_eval(const int& nb_points, Eigen::MatrixXd &result) const{
        result = Eigen::MatrixXd::Zero(nb_points, 3);
        for (int i=0; i<nb_points; i++){
            double t = bounds.first + i * (bounds.second-bounds.first)/(nb_points-1 + EPSILON_APPROX);
            Eigen::Vector3d row;
            eval(t, row);
            result.row(i) = row.transpose();
        }
    }

    virtual void linspace_eval(const int& nb_points, Eigen::MatrixXd &result, std::deque<double>& parameters) const{
        result = Eigen::MatrixXd::Zero(nb_points, 3);
        parameters.clear();
        for (int i=0; i<nb_points; i++){
            double t = bounds.first + i * (bounds.second-bounds.first)/(nb_points-1 + EPSILON_APPROX);
            Eigen::Vector3d row;
            eval(t, row);
            parameters.push_back(t);
            result.row(i) = row.transpose();
        }
    }

    virtual void derivative(const double &t, Eigen::Vector3d &result) const
    {
        Eigen::Vector3d res1, res2;
        eval(t, res1);
        eval(t + GlobalState::get_dt(), res2);
        result = (res2 - res1) / GlobalState::get_dt();
    }
    
    virtual void double_derivative(const double &t, Eigen::Vector3d &result) const
    {
        Eigen::Vector3d res1, res2, res3;
        eval(t - GlobalState::get_dt(), res1);
        eval(t, res2);
        eval(t + GlobalState::get_dt(), res3);
        result = (res1 - 2*res2 + res3) / (GlobalState::get_dt() * GlobalState::get_dt());
    }

    virtual void curvature2D(const double &t, double &signed_curvature, Eigen::Vector3d &direction) const
    {
        Eigen::Vector3d der, dder;
        derivative(t, der);
        double_derivative(t, dder);
        const double norm_der = der.norm();

        if (norm_der < EPSILON_APPROX)
        { // derivative is too small: no use here. The point can be considered as having a singularity
            signed_curvature = 0;
            direction = Eigen::Vector3d::Zero();
            return;
        }

        const double cross = der(0) * dder(1) - der(1) * dder(0);
        const double angle_rot = cross > 0 ? M_PI_2 : -M_PI_2;

        signed_curvature = cross / (norm_der * norm_der * norm_der);
        Eigen::MatrixXd Rot = Eigen::Matrix3d::Zero();
        rotationMatrix2D(angle_rot, Rot);
        direction = (Rot * der) / norm_der; 
    }

    virtual void curvature2D(const double &t, double &signed_curvature) const {
        Eigen::Vector3d direction;
        curvature2D(t, signed_curvature, direction);
    }


    virtual void linspace_eval_curvature(const int& nb_points, Eigen::VectorXd& curvatures, Eigen::MatrixXd &directions) const{
        curvatures = Eigen::VectorXd::Zero(nb_points);
        directions = Eigen::MatrixXd::Zero(nb_points, 3);
        for (int i=0; i<nb_points; i++){
            double t = bounds.first + 2*GlobalState::get_dt() + i * (bounds.second-bounds.first - 4*GlobalState::get_dt())/(nb_points-1);
            Eigen::Vector3d direction;
            double curvature;
            curvature2D(t, curvature, direction);
            curvatures(i) = curvature;
            directions.row(i) = direction.transpose();
        }
    }

    void trim(const double& a, const double& b){
        if (a < bounds.first){M_Log(a, ""); M_Log(bounds.first, ""); throw std::runtime_error("Trim too low.");}
        if (b > bounds.second){M_Log(b, ""); M_Log(bounds.second, ""); throw std::runtime_error("Trim too high.");}
        if (a > b){M_Log(a, ""); M_Log(b, ""); throw std::runtime_error("Trimming bounds are not in the right order.");}
        bounds.first=a;
        bounds.second=b;
    }
    void trim_left(const double& a){
        trim(a, bounds.second);
    }
    void trim_right(const double& b){
        trim(bounds.first, b);
    }
};


class EmptyCurve : public Curve
{
    // This class is provided as an example more than for real use.
private:
    Eigen::Vector3d p;

public:
    EmptyCurve()
    {
        bounds = std::make_pair<double, double>(0., 0.);
        p = Eigen::Vector3d::Zero();
    }

    EmptyCurve(const Eigen::Vector3d& m_p)
    {
        bounds = std::make_pair<double, double>(0., 0.);
        p=m_p;
    }
    ~EmptyCurve(){};

    void eval(const double &t, Eigen::Vector3d &result) const override
    {
        result = p;
    }

    void derivative(const double &t, Eigen::Vector3d &result) const override
    {
        result << 0, 0, 0;
    }

    void double_derivative(const double &t, Eigen::Vector3d &result) const override
    {
        result << 0, 0, 0;
    }
};

class LinearCurve : public Curve
{
    // This class is provided as an example more than for real use.
private:
    Eigen::Vector3d start, end;

public:
    LinearCurve()
    {
        bounds = std::make_pair<double, double>(0., 1.);
        start = Eigen::Vector3d::Zero();
        end = Eigen::Vector3d::Zero();
        end << 1, 1, 0;
    }

    LinearCurve(const Eigen::Vector3d& m_start, const Eigen::Vector3d& m_end)
    {
        bounds = std::make_pair<double, double>(0., 1.);
        start = m_start;
        end = m_end;
    }
    ~LinearCurve(){};

    void eval(const double &t, Eigen::Vector3d &result) const override
    {
        M_Assert(bounds.first <= t and t <= bounds.second, "Value inside the bounds");
        result = start * (1-t) + end*t;
    }

    void derivative(const double &t, Eigen::Vector3d &result) const override
    {
        result = (end-start);
    }

    void double_derivative(const double &t, Eigen::Vector3d &result) const override
    {
        result << 0, 0, 0;
    }
};

class Bezier_Quadratic : public Curve{
    // This class is provided as an example more than for real use.
private:
    Eigen::Vector3d p0, p1, p2;

public:
    Bezier_Quadratic()
    {
        bounds = std::make_pair<double, double>(0., 1.);
        p0 = Eigen::Vector3d::Zero();
        p1 = Eigen::Vector3d::Zero();
        p2 = Eigen::Vector3d::Zero();
        p1 << 0.5, 1, 0;
        p2 << 1, 0, 0;
    }

    Bezier_Quadratic(const Eigen::Vector3d& m_p0, const Eigen::Vector3d& m_p1, const Eigen::Vector3d& m_p2)
    {
        bounds = std::make_pair<double, double>(0., 1.);
        p0 = m_p0;
        p1 = m_p1;
        p2 = m_p2;
    }
    ~Bezier_Quadratic(){};

    void eval(const double &t, Eigen::Vector3d &result) const override
    {
        M_Assert(bounds.first <= t and t <= bounds.second, "Value inside the bounds");
        result = p0 * (1-t)*(1-t) + p1*2*t*(1-t) + p2*t*t;
    }
};