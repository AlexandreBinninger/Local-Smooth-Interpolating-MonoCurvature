#include "view_curve.h"

void get_line_connectivity(const Eigen::MatrixXd &points, const bool &cycling, Eigen::MatrixXi &E)
{
    // assume that points are in the right order
    // get the connectivity of the points for the method set_edges of the viewer
    const int n_points = points.rows();
    const int n_edges = n_points + (cycling ? 0 : -1); // there are as many edges as points if it's cycling, but n-1 edges otherwise
    E = Eigen::MatrixXi(n_edges, 2);
    for (int i = 0; i < n_edges; i++)
    {
        E.row(i) << i, (i + 1) % n_points;
    }
}

void get_endpoints(const Eigen::MatrixXd &points, Eigen::MatrixXd &P1, Eigen::MatrixXd &P2)
{
    P1 = Eigen::MatrixXd(points.rows() - 1, 3);
    P2 = Eigen::MatrixXd(points.rows() - 1, 3);
    for (int i = 0; i < points.rows() - 1; i++)
    {
        P1.row(i) = points.row(i);
        P2.row(i) = points.row(i + 1);
    }
}

void hacky_thick_line_twopoints(const Viewer &viewer, const Eigen::RowVectorXd &P1, const Eigen::RowVectorXd &P2, const int &lineWidth, Eigen::MatrixXd &P)
{
    // inspired from https://github.com/libigl/libigl/issues/1334#issuecomment-914976757
    Eigen::RowVectorXd P1_P2_vec = P2 - P1;
    const int numPoints = P1_P2_vec.norm() * viewer.core().camera_base_zoom * viewer.core().camera_zoom / ((double)lineWidth / 50) + 1; //1000; //P1_P2_vec.norm() / 2 + 1;
    const float distBetweenPoints = 1 / (float)numPoints;
    P = Eigen::MatrixXd::Zero(numPoints, 3);
    for (int i = 0; i < numPoints; i++)
    {
        P.row(i) = P1 + i * distBetweenPoints * P1_P2_vec;
    }
}

void hacky_thick_line(const Viewer &viewer, const Eigen::MatrixXd &points, const Eigen::MatrixXi &E, const int &lineWidth, Eigen::MatrixXd &P)
{
    // inspired from https://github.com/libigl/libigl/issues/1334#issuecomment-914976757
    const int nb_lines = E.rows();
    std::vector<Eigen::MatrixXd> local_P_lines;
    int total_points = 0;
    for (int i = 0; i < nb_lines; i++)
    {
        Eigen::MatrixXd line_P;
        Eigen::RowVectorXd P1 = points.row(E(i, 0));
        Eigen::RowVectorXd P2 = points.row(E(i, 1));
        hacky_thick_line_twopoints(viewer, P1, P2, lineWidth, line_P);
        local_P_lines.push_back(line_P);
        total_points += line_P.rows();
    }

    P = Eigen::MatrixXd::Zero(total_points, points.cols());
    int passed_points = 0;
    for (auto it = local_P_lines.begin(); it != local_P_lines.end(); it++)
    {
        Eigen::MatrixXd line_P = *it;
        const int n_rows = line_P.rows();
        P.block(passed_points, 0, n_rows, line_P.cols()) = line_P;
        passed_points += n_rows;
    }
}

void View::set_curve(Viewer &viewer, const int &id_viewer, const Eigen::MatrixXd &points, const Eigen::MatrixXf &color, const int &lineWidth)
{
    const int n_rows = points.rows(), n_cols = points.cols();
    if (n_rows == 0)
    {
        viewer.data(id_viewer).clear();
        return;
    }
    M_Assert((n_cols == 2) || (n_cols == 3), "Points should be in R^2 or R^3. Point Matrix must therefore be nx2 or nx3");

    Eigen::MatrixXd points_3d;
    if (n_cols == 2)
    {
        points_3d = Eigen::MatrixXd(n_rows, 3);
        Eigen::MatrixXd zero_column = Eigen::MatrixXd::Zero(n_rows, 1);
        points_3d << points, zero_column;
    }
    else
    {
        points_3d = points;
    }
    // get the connectivity for set_edges

    Eigen::MatrixXd P1, P2;
    get_endpoints(points_3d, P1, P2);
    const double scale = GlobalState::get_diagonal_length();
    viewer.data(id_viewer).clear();
    display_lines(viewer, id_viewer, P1, P2, color, (lineWidth * scale) / (GlobalState::view->get_ratio_line()));
}

void View::display_spline(const Eigen::MatrixXd &points)
{
    Eigen::MatrixXf cur_colors = spline_colors.row(0);
    set_curve(GlobalState::getViewer(), GlobalState::id_spline, points, cur_colors, lineWidth);
}

// Put all the values between 0 and 1 with a clamp of clamp_multiplier times the median value.
void median_vector_clamp(std::vector<float> vector_in, std::vector<float>& vector_out, float clamp_multiplier){
    vector_out = vector_in;
    std::sort(vector_in.begin(), vector_in.end());
    const int size = vector_in.size();
    float median = size%2 == 1 ? vector_in[size/2] : (vector_in[size/2] + vector_in[(size/2) + 1])/2;
    for (auto it = vector_out.begin(); it != vector_out.end(); it++){
        if (*it > median * clamp_multiplier){
            *it = 1;
        } else{
            *it = (*it)/(clamp_multiplier*median);
        }
    }
}

void View::display_curvature(const Eigen::MatrixXd &line_points, const Eigen::MatrixXd &curvature_points)
{

    Eigen::MatrixXf cur_colors;
    const Eigen::MatrixXd printed_points = line_points + curvature_points;
    const Eigen::RowVector4f col_growth(1, -0.2, -1, 0);
    const Eigen::RowVector4f col_base(0, 0.5, 1, 1);
    float max = 0;
    if (use_height_curvature_color)
    {
        cur_colors = Eigen::MatrixXf::Zero(printed_points.rows(), 4);
        std::vector<float> norms, norms_clamped;
        for (int i = 0; i < printed_points.rows(); i++)
        {
            float t = (float)curvature_points.row(i).norm();
            norms.push_back(t);
            max = t > max ? t : max;
            cur_colors.row(i) = t * col_growth;
        }
        cur_colors /= max;
        cur_colors.rowwise() += col_base;

        median_vector_clamp(norms, norms_clamped, 2);


        for (int i = 0; i < printed_points.rows(); i++)
        {
            cur_colors.row(i) = norms_clamped[i] * col_growth + col_base;
        }
    }
    else
    {
        cur_colors = curvature_colors.row(0);
    }
    // get the line from point to curvature here

    set_curve(GlobalState::getViewer(), GlobalState::id_curvature, printed_points, cur_colors, lineWidth / 3);

    if (point_to_curvature > 0)
    {
        Eigen::MatrixXd V = GlobalState::getViewer().data(GlobalState::id_curvature).V;
        Eigen::MatrixXi F = GlobalState::getViewer().data(GlobalState::id_curvature).F;
        Eigen::MatrixXd C = GlobalState::getViewer().data(GlobalState::id_curvature).V_material_ambient * 10; // somehow a trick here

        const int nb_new_points = line_points.rows() / point_to_curvature;
        Eigen::MatrixXf cur_point_colors = Eigen::MatrixXf::Zero(nb_new_points, 4);
        Eigen::MatrixXd P1(nb_new_points, 3), P2(nb_new_points, 3);
        for (int i = 0; i < nb_new_points; i++)
        {
            P1.row(i) = line_points.row(point_to_curvature * i);
            P2.row(i) = printed_points.row(point_to_curvature * i);
            cur_point_colors.row(i) = cur_colors.row((point_to_curvature * i)%cur_colors.rows());
        }
        // get new lines
        Eigen::MatrixXd V_new_points;
        Eigen::MatrixXi F_new_points;
        Eigen::MatrixXf C_new_points;
        Eigen::MatrixXf new_colors;
        format_color<float>(cur_point_colors, P1.rows(), new_colors);
        line_square(P1, P2, (lineWidth * GlobalState::get_diagonal_length()) / (GlobalState::view->get_ratio_line()) / 3, new_colors, V_new_points, F_new_points, C_new_points);

        //update previous lines
        int old_nb_points = V.rows(), old_nb_faces = F.rows();
        V.conservativeResize(old_nb_points + V_new_points.rows(), V.cols());
        V.block(old_nb_points, 0, V_new_points.rows(), V.cols()) = V_new_points;
        F.conservativeResize(old_nb_faces + F_new_points.rows(), F.cols());
        for (int i = 0; i < F_new_points.rows(); i++)
        {
            F.row(i + old_nb_faces) << F_new_points(i, 0) + old_nb_points, F_new_points(i, 1) + old_nb_points, F_new_points(i, 2) + old_nb_points;
        }
        C.conservativeResize(old_nb_points + C_new_points.rows(), C.cols());
        for (int i = 0; i < C_new_points.rows(); i++)
        {
            C.row(i + old_nb_points) << C_new_points.row(i).cast<double>();
        }

        GlobalState::getViewer().data(GlobalState::id_curvature).clear();
        GlobalState::getViewer().data(GlobalState::id_curvature).set_mesh(V, F);
        GlobalState::getViewer().data(GlobalState::id_curvature).set_colors(C);
    }
}

void View::display_lasso(const std::vector<Eigen::Vector3d> &lasso_points)
{
    Eigen::MatrixXd points(lasso_points.size(), 3);
    for (int i = 0; i < lasso_points.size(); i++)
    {
        points.row(i) = lasso_points[i].transpose();
    }
    set_curve(GlobalState::getViewer(), GlobalState::id_lasso, points, lasso_color, lineWidth / (2.));
}


void View::display_polygon(const Eigen::MatrixXd& control_points, const bool & cycle){
    Eigen::MatrixXd points = control_points;
    if (cycle){
        points = Eigen::MatrixXd::Zero(control_points.rows() + 1, control_points.cols());
        points.block(0, 0, control_points.rows(), control_points.cols()) = control_points;
        points.row(control_points.rows()) = control_points.row(0);
    }
    set_curve(GlobalState::getViewer(), GlobalState::id_control_polygon, points, polygon_color, lineWidth / (2.));
}