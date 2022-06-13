#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <Eigen/Core>

void read_input(std::string filename, Eigen::MatrixXd &points)
{
    if (filename.empty()){
        points = Eigen::MatrixXd(0, 0); 
    }
    std::ifstream infile(filename);
    double a, b;
    std::vector<Eigen::RowVector3d> points_vector;
    while (infile >> a >> b)
    {
        Eigen::RowVector3d point(a, b, 0);
        points_vector.push_back(point);
    }
    points.resize(points_vector.size(), 3);
    int count = 0;
    for (auto it = points_vector.begin(); it != points_vector.end(); it++){
        points.row(count) = *it;
        count++;
    }
}