// LibIGL libraries
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

// Personal libraries. We used a Model-View-Controller pattern here.
// GLOBAL
#include "global.h"

// MODEL
#include "model/init_model.h"

// VIEW
#include "view/init_view.h"
#include "view/draw.h"
#include "view/view_points.h"

// CONTROLLER
#include "controller/init_controller.h"
#include "controller/mouse_event.h"
#include "controller/keyboard_event.h"

// UTILS
#include "utils/rotation.h"
#include "utils/read_input.h"

Eigen::MatrixXd GlobalState::Input_Points = Eigen::MatrixXd();

int main(int argc, char *argv[])
{
  // Global Initialization
  GlobalState gs = GlobalState::instance();
  *(GlobalState::ROTATION_PI_2) << 0, -1, 0, 1, 0, 0, 0, 0, 0;

  std::string filename;
  if (argc == 2)
  {
    filename = std::string(argv[1]);
  }
  else
  {
    filename = std::string("");
  }
  read_input(filename, GlobalState::Input_Points);

  // Initialize the viewer
  Viewer &viewer = gs.getViewer();
  initialize_viewer();

  // Initialize the model
  initialize_model();

  // Attach a menu plugin
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&menu);
  initialize_menu(menu);
  init_mouse_events();
  init_keyboard_events();

  viewer.callback_pre_draw = draw;
  GlobalState::activate_update();

  viewer.launch(true, false, "Smooth Interpolating Curves with Local Control and Monotonous Alternating Curvature", 1500, 960);
  return EXIT_SUCCESS;
}