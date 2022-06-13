#include "view_tangents.h"


void View::display_tangents(const Eigen::MatrixXd& points, const Eigen::MatrixXd& tangents){
    const Eigen::MatrixXd P1 = points;
    const Eigen::MatrixXd P2 = points + length_tangents*tangents;

    const int id_viewer = GlobalState::id_tangent;
    GlobalState::viewer->data(id_viewer).clear();
    Eigen::MatrixXd V, C;
    Eigen::MatrixXi F;

    Eigen::MatrixXd colors_formatted;
    format_color<double>(GlobalState::view->tangents_color_control_points, P1.rows(), colors_formatted);


    hedra::line_cylinders(P1, P2, GlobalState::view->get_linewidth_world(lineWidth/3), colors_formatted.block(0, 0, colors_formatted.rows(), 3), GlobalState::view->cylinder_resolution, V, F, C);

    GlobalState::viewer->data(id_viewer).set_mesh(V, F);
    GlobalState::viewer->data(id_viewer).set_colors(C);
}