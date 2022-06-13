#pragma once

#include "../global.h"
#include "view.h"
#include "view_line.h"

// Points must be given in an ordered way.
void display_spline(Viewer &viewer, const Eigen::MatrixXd& points, const Eigen::RowVector4f& color = Eigen::RowVector4f(1.0, 1.0, 1.0, 1.0), const int& lineWidth = 15);