#pragma once

// These functions provide "standard models", meaning that they can be used for tests or initialization.
// They are always presented as follows: first the Points variable to be filled. Then, additionnal parameters.

#include "../global.h"
#include "curve.h"
#include "../utils/polygons.h"
#include <random>

class Standard_Models{
    private:

    public:

    enum Model_Type{
        LINE,
        CIRCLE,
        STAR,
        HEART,
        RANDOM,
        POLYGON_RANDOM,
        CONVEXHULL_RANDOM,
        BOX_RANDOM,
        COMPARE1,
        COMPARE2,
        OMEGA,
        OMEGA_SIMPLE,
        INPUT
    };

    static Model_Type model_type;

    static int number_points;
    static double scale_model;

    static void get_standard_model(Eigen::MatrixXd &Points, const double scale=1.0, const int nb_points=5);

    static void line  (Eigen::MatrixXd &Points, const Eigen::Vector3d& start = Eigen::Vector3d(0., 0., 0.), const Eigen::Vector3d& end = Eigen::Vector3d(1., 1., 0.), const double scale = 1.0, const int nb_points = 64);
    static void circle(Eigen::MatrixXd &Points, const double scale=1.0, const int nb_points = 5);
    static void star  (Eigen::MatrixXd &Points, const double scale=1.0, const int nb_branch = 5);
    static void heart (Eigen::MatrixXd &Points, const double scale=1.0, const int nb_points = 5);
    static void random(Eigen::MatrixXd &Points, const double scale=1.0, const int nb_points = 5);
    static void box_random(Eigen::MatrixXd &Points, const double scale=1.0, const int nb_points = 5, const double scale_down = 0.99);
    static void convexhull_random(Eigen::MatrixXd &Points, const double scale=1.0, const int nb_points = 5, const double scale_down = 0.99);
    static void polygon_random(Eigen::MatrixXd &Points, const double scale=1.0, const int nb_points = 5);
    static void compare1(Eigen::MatrixXd &Points, const double& scale);
    static void compare2(Eigen::MatrixXd &Points, const double& scale);
    static void omega(Eigen::MatrixXd &Points, const double& scale);
    static void omega_simple(Eigen::MatrixXd &Points, const double& scale);
    static void input_points(Eigen::MatrixXd &Points);
};