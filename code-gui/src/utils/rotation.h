#pragma once

#include <math.h>
#include <assert.h> /* assert */

#include <Eigen/Core>
#include <igl/rotation_matrix_from_directions.h>

// The rotation matrix size must be initialized: it returns a rotation matrix of same dimension 2x2 or 3x3
void rotationMatrix2D(const double &angle, Eigen::MatrixXd &Rot);
