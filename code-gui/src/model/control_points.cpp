#include "control_points.h"

ControlPoints::ControlPoints()
{
    constraints = std::unique_ptr<Constraints>(new Constraints);
    points = Eigen::MatrixXd(0, 3);
}

void from_2D_to_3D(const Eigen::MatrixXd &points2d, Eigen::MatrixXd &points3d)
{
    points3d = Eigen::MatrixXd::Zero(points2d.rows(), 3);
    points3d.block(0, 0, points2d.rows(), 2) = points2d;
}

// To format the input in the right shape
void format(const Eigen::MatrixXd &points, Eigen::MatrixXd &points3d)
{
    M_Assert((points.cols() == 3 || points.cols() == 2) || ((points.cols() == 1) && (points.rows() == 3 || points.rows() == 2)), "Can add rowvectors, or a 2D/3D set of vectors stacked in a matrix");
    if (points.cols() == 2)
        from_2D_to_3D(points, points3d);
    else if (points.cols() == 3)
        points3d = points;
    else
    {
        if (points.rows() == 2)
            from_2D_to_3D(points.transpose(), points3d);
        else
            points3d = points.transpose();
    }

    M_Assert(points3d.cols() == 3, "Just to be sure.");
}

ControlPoints::ControlPoints(const Eigen::MatrixXd &m_points)
{
    set_points(m_points);
    constraints = std::unique_ptr<Constraints>(new Constraints(m_points.rows()));
}

ControlPoints::ControlPoints(const Eigen::MatrixXd &m_points, const Eigen::MatrixXd& m_tangents, const Eigen::VectorXd& m_curvatures)
{
    set_points(m_points);
    tangents = m_tangents;
    curvatures = m_curvatures;
    constraints = std::unique_ptr<Constraints>(new Constraints(m_points.rows()));
}

void ControlPoints::add_points(const Eigen::MatrixXd &newPoints, int index)
{
    Eigen::MatrixXd addedPoints;
    format(newPoints, addedPoints);

    const int current_nb_points = size();
    if (index <= -1) index = current_nb_points-1;

    points.conservativeResize(current_nb_points + addedPoints.rows(), 3);
    const Eigen::MatrixXd to_move = points.block(index+1, 0, current_nb_points-1-index, 3);
    points.block(index+1, 0, addedPoints.rows(), 3) = addedPoints;
    points.block(index+1+addedPoints.rows(), 0, current_nb_points-1-index, 3) = to_move;
    constraints->reset(points.rows());
}


void ControlPoints::remove_point(int index){
    Eigen::MatrixXd old_points = points;
    const int previous_nb_points = old_points.rows();
    points.resize(points.rows()-1, 3);
    points.block(0, 0, index, 3) = old_points.block(0, 0, index, 3);
    points.block(index, 0, previous_nb_points-1-index, 3) = old_points.block(index+1, 0, previous_nb_points-1-index, 3);
    constraints->reset(points.rows());
}

void ControlPoints::move_point(const Eigen::VectorXd &m_points, int index){
    Eigen::MatrixXd addedPoints;
    format(m_points, addedPoints);
    points.row(index) = addedPoints.row(0);
}

void ControlPoints::clear(){
    points = Eigen::MatrixXd();
    constraints->reset(points.rows());
}


void ControlPoints::set_points(const Eigen::MatrixXd &newPoints)
{
    points = Eigen::MatrixXd(0, 3);
    add_points(newPoints);
    constraints->reset(points.rows());
}



void ControlPoints::set_tangents(const Eigen::MatrixXd& m_tangents){
    tangents = m_tangents;
}

void ControlPoints::set_curvatures(const Eigen::MatrixXd& m_curvatures){
    curvatures = m_curvatures;
}


void ControlPoints::get_points(Eigen::MatrixXd &m_points)
{
    m_points = points;
}


void ControlPoints::get_tangents(Eigen::MatrixXd&  m_tangents)
{
    m_tangents = tangents;
}

void ControlPoints::get_curvatures(Eigen::VectorXd&  m_curvatures)
{
    m_curvatures = curvatures;
}


void ControlPoints::get_point(const int& index, Eigen::RowVector3d &point){
    M_Assert(index >=0 && index<size(), "The index is out of the bounds of the points.");
    point = points.row(index);
}



bool ControlPoints::get_constrained_tangent(const int &index, Eigen::Vector3d &tan)
{
    bool res = constraints->get_tangent_constraints(index, tan);
    if (!res)
    {
        tan = tangents.row(index).transpose();
    }
    return res;
}
bool ControlPoints::get_constrained_curvature(const int &index, double &cur)
{
    bool res = constraints->get_curvature_constraints(index, cur);
    if (!res)
    {
        cur = curvatures(index);
    }
    return res;
}