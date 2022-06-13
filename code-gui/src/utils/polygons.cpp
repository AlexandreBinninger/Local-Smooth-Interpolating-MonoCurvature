#include "polygons.h"

bool intersect_segment(const Eigen::Vector2d &origin, const Eigen::Vector2d &direction, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
{
    const Eigen::Vector2d d = direction.normalized();
    const Eigen::Vector2d v = (p1 - p2);
    const Eigen::Vector2d s = v.normalized();
    if ((direction.squaredNorm() < EPSILON_APPROX) || (abs(d(0) * s(1) - d(1) * s(0)) < EPSILON_APPROX))
    {
        return false;
    }
    // solve A X = B with the right values to find the point of intersection between two straight lines then check that the parameter is in [0, 1] (in the segment)
    Eigen::Matrix2d A;
    A.col(0) = -d;
    A.col(1) = v;
    const Eigen::Vector2d b = origin - p2;
    const Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    const double t = x(1);
    return x(0) > 0 && t >= 0 && t <= 1;
}

bool intersect_2_segments(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2){

    const Eigen::Vector2d u = (p1 - p2);
    const Eigen::Vector2d v = (q1 - q2);
    const Eigen::Vector2d su = u.normalized();
    const Eigen::Vector2d sv = v.normalized();
    if ((u.squaredNorm() < EPSILON_APPROX) || (v.squaredNorm() < EPSILON_APPROX) || (abs(su(0) * sv(1) - su(1) * sv(0)) < EPSILON_APPROX))
    {
        return false;
    }
    // solve A X = B with the right values to find the point of intersection between two straight lines then check that the parameter is in [0, 1] (in the segment)
    Eigen::Matrix2d A;
    A.col(0) = u;
    A.col(1) = -v;
    const Eigen::Vector2d b = q2 - p2;
    const Eigen::Vector2d X = A.colPivHouseholderQr().solve(b);
    const double x = X(0);
    const double t = X(1);
    return x >= 0 && x <= 1 && t >= 0 && t <= 1;
}

void intersect_2_segments(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2, Eigen::Vector2d& intersection_point){
    const Eigen::Vector2d u = (p1 - p2);
    const Eigen::Vector2d v = (q1 - q2);
    const Eigen::Vector2d su = u.normalized();
    const Eigen::Vector2d sv = v.normalized();
    if ((u.squaredNorm() < EPSILON_APPROX) || (v.squaredNorm() < EPSILON_APPROX) || (abs(su(0) * sv(1) - su(1) * sv(0)) < EPSILON_APPROX))
    {
        intersection_point = Eigen::Vector2d::Zero();
        return;
    }
    // solve A X = B with the right values to find the point of intersection between two straight lines then check that the parameter is in [0, 1] (in the segment)
    Eigen::Matrix2d A;
    A.col(0) = u;
    A.col(1) = -v;
    const Eigen::Vector2d b = q2 - p2;
    const Eigen::Vector2d X = A.colPivHouseholderQr().solve(b);
    intersection_point = X(0) * p1 + (1-X(0))*p2;
    
}

bool inside_polygon(const Eigen::Vector3d &point, const std::vector<Eigen::Vector3d> &list_points)
{
    // a point is in a polygon if for any direction, there is an odd number of intersection
    Eigen::Vector2d direction;
    direction << 1, 0;
    const Eigen::Vector2d origin = point.block(0, 0, 2, 1);
    int count = 0;
    const int nb_points = list_points.size();

    for (int i = 0; i < nb_points; i++)
    {
        const Eigen::Vector2d p1 = list_points[i % nb_points].block(0, 0, 2, 1);
        const Eigen::Vector2d p2 = list_points[(i + 1) % nb_points].block(0, 0, 2, 1);
        count += intersect_segment(origin, direction, p1, p2) ? 1 : 0;
    }
    return !!(count % 2);
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
// Basically a cross product
int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r)
{
    int val = (q(1) - p(1)) * (r(0) - q(0)) -
              (q(0) - p(0)) * (r(1) - q(1));

    if (val == 0)
        return 0;             // collinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

void convexhull(const std::vector<Eigen::Vector3d> &list_points, std::vector<Eigen::Vector3d> &convex_hull)
{
    // Initialize Result
    convex_hull.clear();

    const int n = list_points.size();
    // There must be at least 3 points
    if (n < 3)
    {
        convex_hull.insert(convex_hull.begin(), list_points.begin(), list_points.end());
        return;
    }

    // Find the leftmost point
    int l = 0;
    for (int i = 1; i < n; i++)
        if (list_points[i](0) < list_points[l](0))
            l = i;

    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again.  This loop runs O(h)
    // times where h is number of points in result or output.
    int p = l, q;
    do
    {
        // Add current point to result
        convex_hull.push_back(list_points[p]);

        // Search for a point 'q' such that orientation(p, q,
        // x) is counterclockwise for all points 'x'. The idea
        // is to keep track of last visited most counterclock-
        // wise point in q. If any point 'i' is more counterclock-
        // wise than q, then update q.
        q = (p + 1) % n;
        for (int i = 0; i < n; i++)
        {
            // If i is more counterclockwise than current q, then
            // update q
            if (orientation(list_points[p].block(0, 0, 2, 1), list_points[i].block(0, 0, 2, 1), list_points[q].block(0, 0, 2, 1)) == 2)
                q = i;
        }

        // Now q is the most counterclockwise with respect to p
        // Set p as q for next iteration, so that q is added to
        // result 'hull'
        p = q;

    } while (p != l); // While we don't come to first point
}