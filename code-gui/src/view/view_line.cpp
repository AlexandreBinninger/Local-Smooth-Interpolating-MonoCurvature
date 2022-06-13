#include "view_line.h"

void line_square(const Eigen::MatrixXd &P1,
                 const Eigen::MatrixXd &P2,
                 const double &radius,
                 const Eigen::MatrixXf &cyndColors,
                 Eigen::MatrixXd &V,
                 Eigen::MatrixXi &T,
                 Eigen::MatrixXf &C)
{ // inspired from line_cylinders
    M_Assert(P1.rows() == P2.rows(), "line_square requires equality between the two set of corresponding points");
    V.resize(4 * P1.rows(), 3);
    T.resize(2 * P1.rows(), 3);
    int NewColorSize = T.rows();
    C.resize(NewColorSize, cyndColors.cols());

    Eigen::RowVector3d ZAxis;
    ZAxis << 0.0, 0.0, 1.0;
    Eigen::RowVector3d YAxis;
    YAxis << 0.0, 1.0, 0.0;
    for (int i = 0; i < P1.rows(); i++)
    {
        Eigen::RowVector3d NormAxis = (P2.row(i) - P1.row(i)).normalized();
        Eigen::RowVector3d PlaneAxis1 = NormAxis.cross(ZAxis);
        if (PlaneAxis1.norm() < 10e-2)
            PlaneAxis1 = NormAxis.cross(YAxis).normalized(); // this should not happen with (0, 0, 1)- plane but I keep it for generality
        else
            PlaneAxis1 = PlaneAxis1.normalized();
        Eigen::RowVector3d PlaneAxis2 = NormAxis.cross(PlaneAxis1).normalized();
        const int v1 = 4 * i;
        const int v2 = 4 * i + 1;
        const int v3 = 4 * i + 2;
        const int v4 = 4 * i + 3;
        V.row(v1) << P1.row(i) + PlaneAxis1 * radius;
        V.row(v2) << P1.row(i) - PlaneAxis1 * radius;
        V.row(v3) << P2.row(i) + PlaneAxis1 * radius;
        V.row(v4) << P2.row(i) - PlaneAxis1 * radius;

        const int f1 = 2 * i;
        const int f2 = 2 * i + 1;
        T.row(f1) << v3, v2, v1;
        T.row(f2) << v4, v2, v3;

        C.row(f1) << cyndColors.row(i);
        C.row(f2) << cyndColors.row(i);
    }
}

void display_lines(Viewer &viewer, const int &id_viewer, const Eigen::MatrixXd &P1, const Eigen::MatrixXd &P2, const Eigen::MatrixXf &color, const double &width)
{
    M_Assert(P1.rows() == P2.rows(), "Points must have same dimension");
    viewer.data(id_viewer).clear();
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd new_colors, C;
    format_color<double>(color.cast<double>(), P1.rows(), new_colors);
    hedra::line_cylinders(P1, P2, width, new_colors.block(0, 0, new_colors.rows(), 3), GlobalState::view->cylinder_resolution, V, F, C);
    viewer.data(id_viewer).set_mesh(V, F);
    viewer.data(id_viewer).set_colors(C);
}

void add_lines(Viewer &viewer, const int &id_viewer, const Eigen::MatrixXd &P1, const Eigen::MatrixXd &P2, const Eigen::MatrixXf &color, const double &width)
{
    M_Assert(P1.rows() == P2.rows(), "Points must have same dimension");
    if (viewer.data(id_viewer).V.rows() == 0)
    {
        display_lines(viewer, id_viewer, P1, P2, color, width);
        return;
    }
    Eigen::MatrixXd V_new, C;
    Eigen::MatrixXi F_new;
    Eigen::MatrixXd new_colors;
    format_color<double>(color.cast<double>(), P1.rows(), new_colors);
    hedra::line_cylinders(P1, P2, width, new_colors.block(0, 0, new_colors.rows(), 3), GlobalState::view->cylinder_resolution, V_new, F_new, C);

    Eigen::MatrixXd V = viewer.data(id_viewer).V;
    Eigen::MatrixXi F = viewer.data(id_viewer).F;

    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;

    igl::combine(std::vector<Eigen::MatrixXd>{V, V_new}, std::vector<Eigen::MatrixXi>{F, F_new}, V_out, F_out);
    viewer.data(id_viewer).clear();
    viewer.data(id_viewer).set_mesh(V_out, F_out);
    viewer.data(id_viewer).set_colors(color.cast<double>());
}