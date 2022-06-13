#include "standard_models.h"

Standard_Models::Model_Type Standard_Models::model_type = Standard_Models::Model_Type::STAR;
int Standard_Models::number_points = 10;

double Standard_Models::scale_model = 5.0;

std::random_device rd;
std::mt19937 re = std::mt19937(rd());
std::uniform_real_distribution<double> unif_0_1 = std::uniform_real_distribution<double>(0., 1.);

void Standard_Models::get_standard_model(Eigen::MatrixXd &Points, const double scale, const int nb_points)
{
    switch (model_type)
    {
    case (LINE):
    {
        line(Points, Eigen::Vector3d(-scale / (sqrt(2)), -scale / (sqrt(2)), 0.), Eigen::Vector3d(scale / (sqrt(2)), scale / (sqrt(2)), 0.), scale, nb_points);
    }
    break;
    case (CIRCLE):
    {
        circle(Points, scale, nb_points);
    }
    break;
    case (STAR):
    {
        star(Points, scale, std::max(nb_points / 2, 1));
    }
    break;
    case (HEART):
    {
        heart(Points, scale, nb_points);
    }
    break;
    case (RANDOM):
    {
        random(Points, scale, nb_points);
    }
    break;
    case (POLYGON_RANDOM):
    {
        polygon_random(Points, scale, nb_points);
    }
    break;
    case (CONVEXHULL_RANDOM):
    {
        convexhull_random(Points, scale, nb_points);
    }
    break;
    case (BOX_RANDOM):
    {
        box_random(Points, scale, nb_points);
    }
    break;
    case (COMPARE1):
    {
        compare1(Points, scale);
    }
    break;
    case (COMPARE2):
    {
        compare2(Points, scale);
    }
    break;
    case (OMEGA):
    {
        omega(Points, scale);
    }
    break;
    case (OMEGA_SIMPLE):
    {
        omega_simple(Points, scale);
    }
    break;
    case (INPUT):
    {
        input_points(Points);
    }
    break;
    default:
        break;
    }
}

void Standard_Models::line(Eigen::MatrixXd &Points, const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double scale, const int nb_points)
{
    M_Assert(nb_points >= 1, "At least 1 point!");
    Eigen::VectorXd linspace = Eigen::VectorXd::LinSpaced(nb_points, 0., 1.);
    LinearCurve Lcu;
    Points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i = 0; i < linspace.rows(); i++)
    {
        Eigen::Vector3d p;
        Lcu.eval(linspace(i), p);
        Points.row(i) = p.transpose();
    }
}

void Standard_Models::circle(Eigen::MatrixXd &Points, const double scale, const int nb_points)
{
    M_Assert(nb_points >= 1, "At least 1 point!");
    Points = Eigen::MatrixXd::Zero(nb_points, 3);
    double angle = 0;
    const double angle_step = 2 * M_PI / nb_points;
    for (int i = 0; i < Points.rows(); i++)
    {
        Points.row(i) << cos(angle), sin(angle), 0;
        angle += angle_step;
    }
    Points *= scale;
}

void Standard_Models::star(Eigen::MatrixXd &Points, const double scale, const int nb_branch)
{
    M_Assert(nb_branch >= 1, "At least 1 point!");
    Points = Eigen::MatrixXd::Zero(2 * nb_branch, 3);
    double angle = M_PI_2;
    const double angle_step = M_PI / nb_branch; // 2pi / (2n)
    for (int i = 0; i < nb_branch; i++)
    {
        Points.row(2 * i) << cos(angle), sin(angle), 0;
        angle += angle_step;
        Points.row(2 * i + 1) << cos(angle) / 2, sin(angle) / 2, 0;
        angle += angle_step;
    }
    Points *= scale;
}

void Standard_Models::heart(Eigen::MatrixXd &Points, const double scale, const int nb_points)
{
    M_Assert(nb_points >= 1, "At least 1 point!");
    //Comes from there: https://blogs.lcps.org/academiesonline/2021/02/13/the-equation-of-the-heart/
    Eigen::VectorXd linspace = Eigen::VectorXd::LinSpaced(nb_points + 1, 0., 2 * M_PI);
    Points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i = 0; i < nb_points; i++)
    {
        const double t = linspace(i);
        const double x = 16 * pow(sin(t), 3);
        const double y = 13 * cos(t) - 5 * cos(2 * t) - 2 * cos(3 * t) - cos(4 * t);
        Points.row(i) << x, y, 0;
    }
    Points.normalize();
    Points = Points * scale;
}

void Standard_Models::random(Eigen::MatrixXd &Points, const double scale, const int nb_points)
{
    M_Assert(nb_points >= 1, "At least 1 point!");
    Points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i = 0; i < nb_points; i++)
    {
        Points.row(i) << unif_0_1(re), unif_0_1(re), 0;
    }
    Points *= scale;
}

void Standard_Models::polygon_random(Eigen::MatrixXd &Points, const double scale, const int nb_points)
{
    /* This function creates points randomly such that the line between the previous point and the new point does not intersect any other line of consecutive */
    M_Assert(nb_points >= 1, "At least 1 point!");
    std::vector<Eigen::Vector3d> list_random_points;
    Eigen::Vector3d p;
    p << unif_0_1(re), unif_0_1(re), 0;
    list_random_points.push_back(p);
    p << unif_0_1(re), unif_0_1(re), 0;
    list_random_points.push_back(p);
    for (int i = 2; i < nb_points; i++)
    {
        const Eigen::Vector2d o = (*(list_random_points.rbegin())).block(0, 0, 2, 1);
        bool intersection = true;
        while (intersection)
        {
            intersection = false;
            p << unif_0_1(re), unif_0_1(re), 0;
            const Eigen::Vector2d d = (p.block(0, 0, 2, 1) - o);
            for (int j = 0; !intersection && (j < list_random_points.size() - 2); j++)
            {
                const Eigen::Vector2d p1 = list_random_points[j].block(0, 0, 2, 1);
                const Eigen::Vector2d p2 = list_random_points[j + 1].block(0, 0, 2, 1);
                intersection = intersection || intersect_2_segments(o, p.block(0, 0, 2, 1), p1, p2);
            }
        }
        list_random_points.push_back(p);
    }

    Points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i = 0; i < nb_points; i++)
    {
        Points.row(i) = list_random_points[i].transpose();
    }
    Points *= scale;
}

void Standard_Models::convexhull_random(Eigen::MatrixXd &Points, const double scale, const int nb_points, const double scale_down)
{
    /* This function creates points randomly such that each new point is not in the convex hull of the current shape 
    and the new line does not cross the convex hull.
    */
    M_Assert(nb_points >= 1, "At least 1 point!");
    std::vector<Eigen::Vector3d> list_random_points, list_convex_hull;
    Eigen::Vector3d p;
    p << unif_0_1(re), unif_0_1(re), 0;
    list_random_points.push_back(p);
    list_convex_hull.push_back(p);
    p << unif_0_1(re), unif_0_1(re), 0;
    list_random_points.push_back(p);
    list_convex_hull.push_back(p);
    double scale_new_point = 1.0;
    for (int i = 2; i < nb_points; i++)
    {
        bool intersection = true;
        while (intersection)
        {
            intersection = false;
            const Eigen::Vector2d o = (*(list_random_points.rbegin())).block(0, 0, 2, 1);
            p << unif_0_1(re), unif_0_1(re), 0;
            p *= scale_new_point;
            intersection = intersection || inside_polygon(p, list_convex_hull);
            const int nb_convex_hull = list_convex_hull.size();
            for (int j = 0; j < nb_convex_hull; j++)
            {
                const Eigen::Vector2d p1 = list_convex_hull[j % nb_convex_hull].block(0, 0, 2, 1);
                const Eigen::Vector2d p2 = list_convex_hull[(j + 1) % nb_convex_hull].block(0, 0, 2, 1);
                intersection = intersection || intersect_2_segments(o, p.block(0, 0, 2, 1), p1, p2);
            }
            if (intersection)
            {
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                for (auto it = list_random_points.begin(); it != list_random_points.end(); it++)
                {
                    center += *it;
                }
                center = center / list_random_points.size();
                for (auto it = list_random_points.begin(); it != list_random_points.end(); it++)
                {
                    *it = (*it - center) * scale_down + center;
                }

                center = Eigen::Vector3d::Zero();
                for (auto it = list_convex_hull.begin(); it != list_convex_hull.end(); it++)
                {
                    center += *it;
                }
                center = center / list_random_points.size();
                for (auto it = list_convex_hull.begin(); it != list_convex_hull.end(); it++)
                {
                    *it = (*it - center) * scale_down + center;
                }
            }
        }
        list_random_points.push_back(p);
        scale_new_point = std::max(scale_new_point, p.norm());
        // recompute new convexhull
        convexhull(list_random_points, list_convex_hull);
    }

    Points = Eigen::MatrixXd::Zero(nb_points, 3);
    for (int i = 0; i < nb_points; i++)
    {
        Points.row(i) = list_random_points[i].transpose();
    }
    Points.normalize();
    Points *= scale;
}

void Standard_Models::box_random(Eigen::MatrixXd &Points, const double scale, const int nb_points, const double scale_down)
{
    /* This function creates points randomly such that each new point is not in the box of the current shape.*/
    M_Assert(nb_points >= 1, "At least 1 point!");
    enum Corner
    {
        TopLeft,
        TopRight,
        BottomLeft,
        BottomRight
    };
    Corner corner;
    Points = Eigen::MatrixXd::Zero(nb_points, 3);
    double min_x, max_x, min_y, max_y;
    Points.row(0) << unif_0_1(re) / 4, unif_0_1(re) / 4, 0;
    if (nb_points == 1)
        return;
    const Eigen::RowVector3d p0 = Points.row(0);
    min_x = p0(0);
    max_x = p0(0);
    min_y = p0(1);
    max_y = p0(1);

    Eigen::RowVector3d p;
    p << unif_0_1(re) / 3, unif_0_1(re) / 3, 0;
    while (p(0) < min_x && p(1) < min_y)
    {
        p << unif_0_1(re) / 3, unif_0_1(re) / 3, 0;
    }
    max_x = p(0);
    max_y = p(1);
    Points.row(1) = p;
    if (nb_points == 2)
        return;

    p << unif_0_1(re) / 2, unif_0_1(re) / 2, 0;
    while (p(0) > min_x && p(0) < max_x && p(1) > min_y && p(1) < max_y)
    {
        p << unif_0_1(re) / 2, unif_0_1(re) / 2, 0;
    }
    Points.row(2) = p;
    if (nb_points == 3)
        return;

    // Determine in which situation we are: bottom/top, left/right
    if (p(0) < min_x)
    {
        min_x = p(0);
        if (p(1) < min_y)
        {
            corner = Corner::BottomLeft;
            min_y = p(1);
        }
        else
        {
            corner = Corner::TopLeft;
            max_y = p(1);
        }
    }
    else
    { //p(0) > max_x
        max_x = p(0);
        if (p(1) < min_y)
        {
            corner = Corner::BottomRight;
            min_y = p(1);
        }
        else
        {
            corner = Corner::TopRight;
            max_y = p(1);
        }
    }

    for (int i = 3; i < nb_points; i++)
    {
        bool is_good = false;

        while (!is_good)
        {
            p << unif_0_1(re), unif_0_1(re), 0;

            // Determine in which situation we are: bottom/top, left/right
            switch (corner)
            {
            case (Corner::BottomLeft):
                is_good = true;
                if (p(0) < min_x && p(1) < min_y)
                {
                    min_x = p(0);
                    min_y = p(1);
                    corner = Corner::BottomLeft;
                }
                else if (p(0) > max_x && p(1) < min_y)
                {
                    max_x = p(0);
                    min_y = p(1);
                    corner = Corner::BottomRight;
                }
                else if (p(0) < min_x && p(1) > max_y)
                {
                    min_x = p(0);
                    max_y = p(1);
                    corner = Corner::TopLeft;
                }
                else
                    is_good = false;
                break;

            case (Corner::TopLeft):
                is_good = true;
                if (p(0) < min_x && p(1) < min_y)
                {
                    min_x = p(0);
                    min_y = p(1);
                    corner = Corner::BottomLeft;
                }
                else if (p(0) > max_x && p(1) > max_y)
                {
                    max_x = p(0);
                    max_y = p(1);
                    corner = Corner::TopRight;
                }
                else if (p(0) < min_x && p(1) > max_y)
                {
                    min_x = p(0);
                    max_y = p(1);
                    corner = Corner::TopLeft;
                }
                else
                    is_good = false;
                break;

            case (Corner::BottomRight):
                is_good = true;
                if (p(0) < min_x && p(1) < min_y)
                {
                    min_x = p(0);
                    min_y = p(1);
                    corner = Corner::BottomLeft;
                }
                else if (p(0) > max_x && p(1) < min_y)
                {
                    max_x = p(0);
                    min_y = p(1);
                    corner = Corner::BottomRight;
                }
                else if (p(0) > max_x && p(1) > max_y)
                {
                    max_x = p(0);
                    max_y = p(1);
                    corner = Corner::TopRight;
                }
                else
                    is_good = false;
                break;

            case (Corner::TopRight):
                is_good = true;
                if (p(0) < min_x && p(1) > max_y)
                {
                    min_x = p(0);
                    max_y = p(1);
                    corner = Corner::TopLeft;
                }
                else if (p(0) > max_x && p(1) < min_y)
                {
                    max_x = p(0);
                    min_y = p(1);
                    corner = Corner::BottomRight;
                }
                else if (p(0) > max_x && p(1) > max_y)
                {
                    max_x = p(0);
                    max_y = p(1);
                    corner = Corner::TopRight;
                }
                else
                    is_good = false;
                break;
            default:
                break;
            }

            //center and scale down points a bit to help a bit the resolution and prevent from infinite run.
            Points.rowwise() -= Eigen::RowVector3d(0.5, 0.5, 0);
            Points *= scale_down;
            Points.rowwise() += Eigen::RowVector3d(0.5, 0.5, 0);
            min_x = (min_x - 0.5) * scale_down + 0.5;
            max_x 
            = (max_x - 0.5) * scale_down + 0.5;
            min_y = (min_y - 0.5) * scale_down + 0.5;
            max_y = (max_y - 0.5) * scale_down + 0.5;
        }

        Points.row(i) = p;
    }
    Points *= scale;
}
void Standard_Models::compare1(Eigen::MatrixXd &Points, const double &scale)
{
    Points = Eigen::MatrixXd(7, 3);

    Points << 596.5180957343301, 607.4701062828644, 0,
        560.2855813656738, 748.2828325792336, 0,
        986.8410914330377, 664.2892765428028, 0,
        924.2576575235402, 700.5217909114592, 0,
        732.3900246167916, 367.84143170834153, 0,
        715.0972336681148, 371.9587628865979, 0,
        457.3523019092636, 601.7058426333055, 0;

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (int i = 0; i < Points.rows(); i++)
    {
        Eigen::RowVector3d old_point = Points.row(i);
        center += old_point.transpose();
    }
    center = center / Points.size();
    double max_norm = 0;
    for (int i = 0; i < Points.rows(); i++)
    {
        Eigen::RowVector3d old_point = Points.row(i);
        Points.row(i) = old_point - center.transpose();
        if (Points.row(i).norm() > max_norm)
        {
            max_norm = Points.row(i).norm();
        }
        Points(i, 1) *= -1;
    }
    Points = scale * Points/max_norm;
}

void Standard_Models::compare2(Eigen::MatrixXd &Points, const double &scale)
{
    Points = Eigen::MatrixXd(6, 3);

    Points << 599.2939632545932, 601.2948381452319, 0,
        780.2213473315836, 479.65004374453196, 0,
        820.2563429571304, 479.65004374453196, 0,
        993.4846894138233, 601.2948381452319, 0,
        1011.1924759405075, 580.5074365704287, 0,
        580.8162729658793, 580.5074365704287, 0;

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (int i = 0; i < Points.rows(); i++)
    {
        Eigen::RowVector3d old_point = Points.row(i);
        center += old_point.transpose();
    }
    center = center / Points.size();
    double max_norm = 0;
    for (int i = 0; i < Points.rows(); i++)
    {
        Eigen::RowVector3d old_point = Points.row(i);
        Points.row(i) = old_point - center.transpose();
        if (Points.row(i).norm() > max_norm)
        {
            max_norm = Points.row(i).norm();
        }
        Points(i, 1) *= -1;
    }
    Points = scale * Points / max_norm;
}

void Standard_Models::omega(Eigen::MatrixXd &Points, const double &scale)
{

    Points = Eigen::MatrixXd::Zero(26, 3);
    Eigen::VectorXd X(26);
    X << 2.9265642,
        2.6734362,
        2.5109322,
        1.9078122,
        1.1859282,
        1.9249962,
        2.8265562,
        0.00468420000000025,
        -2.826567,
        -1.9437558,
        -1.1859438,
        -1.9062558,
        -2.501565,
        -2.6734386,
        -2.9265642,
        -2.6187522,
        -1.1406318,
        -0.8968758,
        -1.4562558,
        -1.9062558,
        -0.00468780000000013,
        1.9078122,
        1.4468682,
        0.8968722,
        1.1406282,
        2.6187522;

    Eigen::VectorXd Y(26);
    Y << -1.707808758,
        -1.707808758,
        -2.367185958,
        -2.582810358,
        -2.582810358,
        -1.167184758,
        0.915619242,
        3.178123242,
        0.915619242,
        -1.150000758,
        -2.582810358,
        -2.582810358,
        -2.393750358,
        -1.707808758,
        -1.707808758,
        -3.178123242,
        -3.178123242,
        -2.989063158,
        -0.915616758,
        0.925003242,
        2.953123242,
        0.925003242,
        -0.915616758,
        -2.989063158,
        -3.178123242,
        -3.178123242;

    Points.col(0) = X;
    Points.col(1) = Y;
    Points *= scale;
}

void Standard_Models::omega_simple(Eigen::MatrixXd &Points, const double &scale)
{

    Points = Eigen::MatrixXd::Zero(18, 3);

    Points << 2.92656, -1.70781, 0,
        2.67344, -1.70781, 0,
        2.51093, -2.36719, 0,
        1.18593, -2.58281, 0,
        2.82656, 0.915619, 0,
        0.0046842, 3.17812, 0,
        -2.82657, 0.915619, 0,
        -1.18594, -2.58281, 0,
        -2.50156, -2.39375, 0,
        -2.67344, -1.70781, 0,
        -2.92656, -1.70781, 0,
        -2.61875, -3.17812, 0,
        -0.896876, -2.98906, 0,
        -2.057, 0.925003, 0,
        -0.0046878, 2.95312, 0,
        2.05856, 0.925003, 0,
        0.896872, -2.98906, 0,
        2.61875, -3.17812, 0;
    Points *= scale;
}

void Standard_Models::input_points(Eigen::MatrixXd &Points)
{
    Points = GlobalState::Input_Points;
    if (Points.rows() == 0){
        star(Points, 5, 5);
    }
}