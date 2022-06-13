#pragma once

#include "../global.h"
#include "circle.h"
#include "rotation.h"

// Set of functions useful for finding the circle passing on three points.

void ellipse_center(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, Eigen::Vector2d& center, Eigen::Vector2d& ax1, Eigen::Vector2d& ax2, double& max_theta, bool& reverse);
void ellipse_center(const Eigen::Matrix3d& points, Eigen::Vector3d& center, Eigen::Vector3d& ax1, Eigen::Vector3d& ax2, double& max_theta, bool& reverse);