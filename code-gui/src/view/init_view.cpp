#include "init_view.h"
int GlobalState::id_plane=0, GlobalState::id_control_points=0, GlobalState::id_spline=0, GlobalState::id_tangent=0, GlobalState::id_curvature=0, GlobalState::id_osculating=0, GlobalState::id_lasso=0, GlobalState::id_control_polygon=0, GlobalState::id_constraint_line=0, GlobalState::id_constraint_circle=0;
std::unique_ptr<Viewer> GlobalState::viewer(new Viewer);
std::unique_ptr<View> GlobalState::view(new View); 


void initialize_viewer()
{
  GlobalState gs = GlobalState::instance();
  Viewer& viewer = GlobalState::instance().getViewer();
  viewer.core().background_color = Eigen::RowVector4f(255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0);
  viewer.core().orthographic = true;
  viewer.core().rotation_type = igl::opengl::ViewerCore::ROTATION_TYPE_NO_ROTATION;
  viewer.core().is_animating = true;
  viewer.core().lighting_factor = 0;

  GlobalState::id_plane = viewer.data().id;
  viewer.data(GlobalState::id_plane).set_face_based(true);
  viewer.core().align_camera_center(viewer.data(GlobalState::id_plane).V, viewer.data(GlobalState::id_plane).F);
  viewer.data(GlobalState::id_plane).show_lines = false;
  viewer.data(GlobalState::id_plane).show_faces = false;

  GlobalState::id_control_points    = viewer.append_mesh(true);
  GlobalState::id_spline            = viewer.append_mesh(true);
  GlobalState::id_tangent           = viewer.append_mesh(GlobalState::view->flag_display_tangents_control_points);
  GlobalState::id_curvature         = viewer.append_mesh(true);
  GlobalState::id_osculating        = viewer.append_mesh(GlobalState::view->flag_display_osculating_circle_control_points);
  GlobalState::id_lasso             = viewer.append_mesh(true);
  GlobalState::id_control_polygon   = viewer.append_mesh(GlobalState::view->display_control_polygon);
  GlobalState::id_constraint_line   = viewer.append_mesh(true);
  GlobalState::id_constraint_circle = viewer.append_mesh(true);
  viewer.data(GlobalState::id_control_points).show_lines    = false;
  viewer.data(GlobalState::id_control_points).double_sided  = false;
  viewer.data(GlobalState::id_spline).show_lines            = false;
  viewer.data(GlobalState::id_spline).double_sided          = true;
  viewer.data(GlobalState::id_tangent).show_lines           = false;
  viewer.data(GlobalState::id_curvature).show_lines         = false;
  viewer.data(GlobalState::id_osculating).show_lines        = false;
  viewer.data(GlobalState::id_lasso).show_lines             = false;
  viewer.data(GlobalState::id_control_polygon).show_lines   = false;
  viewer.data(GlobalState::id_constraint_line).show_lines   = false;
  viewer.data(GlobalState::id_constraint_circle).show_lines = false;
}

