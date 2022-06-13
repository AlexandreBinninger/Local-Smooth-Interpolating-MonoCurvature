#include "rotation.h"

void rotationMatrix2D(const double & angle, Eigen::MatrixXd &Rot){
    assert(Rot.cols() == Rot.rows()); //Rotation matrix is squared
    assert(Rot.cols() == 2 || Rot.cols() == 3); // Rotation matrix is 2x2 or 3x3"
    // If rotation Matrix is 3x3, then it's just the same as 2x2 but with 0 everywhere else exept of R(3, 3)=1. This function is for 2D rotation in the end

    Eigen::Matrix2d Rot_2D;
    const double cos_a = cos(angle);
    const double sin_a = sin(angle);
    Rot_2D << cos_a, -sin_a,
           sin_a, cos_a; 

    if (Rot.cols() ==2) Rot = Rot_2D;
    else{
        Rot = Eigen::Matrix3d::Identity();
        Rot.block(0, 0, 2, 2) = Rot_2D;
    }
} 
