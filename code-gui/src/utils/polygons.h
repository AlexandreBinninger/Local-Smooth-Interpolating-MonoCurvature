#pragma once

#include "../global.h"

// Does the straight line of origin o and direction d intersect the segment p1-p2?
bool intersect_segment(const Eigen::Vector2d& origin, const Eigen::Vector2d& direction, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
bool intersect_2_segments(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
bool inside_polygon(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& list_points);
void intersect_2_segments(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2, Eigen::Vector2d& intersection_point);


// A C++ program to find convex hull of a set of points. Based on
// https://www.geeksforgeeks.org/orientation-3-ordered-points/
// Refer to it for explanation of orientation()
void convexhull(const std::vector<Eigen::Vector3d>& list_points, std::vector<Eigen::Vector3d>& convex_hull);