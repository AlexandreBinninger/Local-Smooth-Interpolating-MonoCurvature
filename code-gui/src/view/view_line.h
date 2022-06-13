#pragma once

#include "../global.h"
#include "view.h"
#include "hedra/line_cylinders.h"
#include "igl//combine.h"

void line_square(const Eigen::MatrixXd &P1,
                 const Eigen::MatrixXd &P2,
                 const double &radius,
                 const Eigen::MatrixXf &cyndColors,
                 Eigen::MatrixXd &V,
                 Eigen::MatrixXi &T,
                 Eigen::MatrixXf &C);
void display_lines(Viewer &viewer, const int& id_viewer, const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2, const Eigen::MatrixXf& color, const double& width);
void add_lines(Viewer &viewer, const int &id_viewer, const Eigen::MatrixXd &P1, const Eigen::MatrixXd &P2, const Eigen::MatrixXf &color, const double &width);