#pragma once

#include "../global.h"

// Set of functions useful for finding the circle passing on three points.

bool are_aligned(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);
bool are_aligned(const Eigen::Matrix3d& points);
void circle_radius_center(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, double &radius, Eigen::Vector2d& center);
void circle_radius_center(const Eigen::Matrix3d& points, double &radius, Eigen::Vector3d& center);