#pragma once
#include "curve.h"
#include "../utils/misc.h"
#include "../utils/polygons.h"
#include "Clothoids.hh"

class Clothoid_Curve : public Curve
{
private:
    G2lib::ClothoidCurve *clotho_curve;
    double x0, y0;
    bool static_point;

public:
    Clothoid_Curve(const Eigen::VectorXd &p0, const Eigen::VectorXd &t0, const Eigen::VectorXd &p1, const Eigen::VectorXd &t1)
    {
        x0 = p0(0), y0 = p0(1);
        const double x1 = p1(1), y1 = p1(1);

        const double theta0 = atan2(t0(1), t0(0));
        const double theta1 = atan2(t1(1), t1(0));
        try
        {
            clotho_curve = new G2lib::ClothoidCurve(p0.data(), theta0, p1.data(), theta1);
            bounds = std::make_pair<double, double>(0.0, clotho_curve->length());
            static_point = false;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            clotho_curve = new G2lib::ClothoidCurve();
            bounds = std::make_pair<double, double>(0., 1.);
            static_point = true;
        }
    }
    Clothoid_Curve()
    {
        clotho_curve = new G2lib::ClothoidCurve();
        bounds = std::make_pair<double, double>(0.0, clotho_curve->length());
    }

    ~Clothoid_Curve()
    {
        delete clotho_curve;
    }

    void eval(const double &t, Eigen::Vector3d &result) const override
    {
        if (static_point)
        {
            M_Log(x0, "");
            M_Log(y0, "");
        }
        M_Assert(bounds.first <= t && t <= bounds.second, "Value inside the bounds");
        double x = 0, y = 0;
        clotho_curve->eval(t, x, y);

        result << x, y, 0;
    }

    double curvature_beginning(){
        return clotho_curve->kappaBegin();
    }

    double curvature_end(){
        return clotho_curve->kappaEnd();
    }
};
