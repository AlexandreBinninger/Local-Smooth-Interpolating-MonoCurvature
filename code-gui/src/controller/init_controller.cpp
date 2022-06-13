#include "init_controller.h"

std::unique_ptr<Controller> GlobalState::controller(new Controller);

void modification()
{
  GlobalState::spline->softreset();
  GlobalState::activate_update();
}

void initialize_menu(igl::opengl::glfw::imgui::ImGuiMenu &menu)
{
  Viewer &viewer = GlobalState::getViewer();
  double v_min_zero = 0;

  static bool display_control_points = true;
  // Add content to the default menu window
  menu.callback_draw_viewer_menu = [&]()
  {
    // Draw parent menu content
    // Viewing options
    if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
      if (ImGui::Button("center object on points", ImVec2(-1, 0)))
      {
        set_plane_control_points();
        Eigen::MatrixXd control_points;
        GlobalState::get_control_points(control_points);
        viewer.core().align_camera_center(control_points);
        viewer.core().camera_translation = Eigen::Vector3f::Zero();
        viewer.core().camera_zoom = 1.f;
        GlobalState::activate_update();
      }

      if (ImGui::Button("center object on curve", ImVec2(-1, 0)))
      {
        set_plane_control_points();
        Eigen::MatrixXd spline_points;
        GlobalState::get_spline_points(spline_points);
        viewer.core().align_camera_center(spline_points);
        viewer.core().camera_translation = Eigen::Vector3f::Zero();
        viewer.core().camera_zoom = 1.f;
        GlobalState::activate_update();
      }

      // Zoom
      ImGui::PushItemWidth(80 * menu.menu_scaling());
      if (ImGui::DragFloat("zoom", &(viewer.core().camera_zoom), 0.05f, 0.1f, 20.0f))
        GlobalState::activate_update();

      ImGui::PopItemWidth();
    }

    // Helper for setting viewport specific mesh options
    auto make_checkbox = [&](const char *label, unsigned int &option)
    {
      return ImGui::Checkbox(
          label,
          [&]()
          { return viewer.core().is_set(option); },
          [&](bool value)
          { return viewer.core().set(option, value); });
    };

    // Draw options
    if (ImGui::CollapsingHeader("Draw Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
      ImGui::ColorEdit4("background", viewer.core().background_color.data(),
                        ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
      ImGui::PopItemWidth();

      // Control Points View
      if (ImGui::CollapsingHeader("Control Points View", ImGuiTreeNodeFlags_DefaultOpen))
      {
        if (ImGui::Checkbox("display points", &display_control_points))
          viewer.data(GlobalState::id_control_points).set_visible(display_control_points);

        if (ImGui::Checkbox("curvature color", &GlobalState::view->show_curvature_color))
        {
          GlobalState::activate_update();
        }

        if (ImGui::InputInt("size", &(GlobalState::view->pointSize)))
        {
          GlobalState::view->pointSize = GlobalState::view->pointSize >= 0 ? GlobalState::view->pointSize : 0;
          GlobalState::activate_update();
        }

        if (ImGui::InputInt("sides", &(GlobalState::view->sphere_side)))
        {
          GlobalState::view->sphere_side = GlobalState::view->sphere_side >= 3 ? GlobalState::view->sphere_side : 3;
          GlobalState::activate_update();
        }

        if (ImGui::ColorEdit3("point color", GlobalState::view->control_points_colors.row(0).data(),
                              ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel))
        {
          GlobalState::view->control_points_colors_negative_curvature(0) = GlobalState::view->control_points_colors(1);
          GlobalState::view->control_points_colors_negative_curvature(1) = GlobalState::view->control_points_colors(0);
          GlobalState::activate_update();
        }
      }
      // Spline View
      // Line
      if (ImGui::CollapsingHeader("Line View", ImGuiTreeNodeFlags_DefaultOpen))
      {

        if (ImGui::Checkbox("display spline", &GlobalState::view->display_splines))
          viewer.data(GlobalState::id_spline).set_visible(GlobalState::view->display_splines);

        if (ImGui::Checkbox("control polygon", &GlobalState::view->display_control_polygon))
          viewer.data(GlobalState::id_control_polygon).set_visible(GlobalState::view->display_control_polygon);
        if (ImGui::InputInt("resolution", &(GlobalState::spline->nb_points_per_curve)))
        {
          GlobalState::spline->nb_points_per_curve = GlobalState::spline->nb_points_per_curve > 2 ? GlobalState::spline->nb_points_per_curve : 2;
          GlobalState::recompute_spline_points();
          GlobalState::activate_update();
        }

        if (ImGui::InputInt("width", &(GlobalState::view->lineWidth)))
        {
          GlobalState::view->lineWidth = GlobalState::view->lineWidth >= 0 ? GlobalState::view->lineWidth : 0;
          GlobalState::activate_update();
        }
        if (ImGui::ColorEdit4("line color", GlobalState::view->spline_colors.row(0).data(),
                              ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel))
        {
          viewer.data(GlobalState::id_spline).line_color = Eigen::RowVector4f::Ones() - GlobalState::view->spline_colors.row(0);
          GlobalState::activate_update();
        }
      }
      if (ImGui::CollapsingHeader("Tangent View", ImGuiTreeNodeFlags_DefaultOpen))
      {
        if (ImGui::Checkbox("display tangents", &GlobalState::view->flag_display_tangents_control_points))
        {
          viewer.data(GlobalState::id_tangent).set_visible(GlobalState::view->flag_display_tangents_control_points);
          GlobalState::activate_update();
        }

        double p_step = 0.01;
        double p_step_fast = 0.1;
        if (ImGui::InputScalar("tangent scale", ImGuiDataType_Double, &GlobalState::view->length_tangents, &p_step, &p_step_fast, "%.2lf"))
        {
          modification();
        }
      }

      if (ImGui::CollapsingHeader("Curvature View", ImGuiTreeNodeFlags_DefaultOpen))
      {
        if (ImGui::Checkbox("display osculating circles", &GlobalState::view->flag_display_osculating_circle_control_points))
        {
          viewer.data(GlobalState::id_osculating).set_visible(GlobalState::view->flag_display_osculating_circle_control_points);
          GlobalState::activate_update();
        }

        if (ImGui::InputInt("resolution osculating", &(GlobalState::view->resolution_osculating)))
        {
          GlobalState::view->resolution_osculating = GlobalState::view->resolution_osculating >= 0 ? GlobalState::view->resolution_osculating : 0;
          GlobalState::activate_update();
        }

        if (ImGui::Checkbox("display curvature", &GlobalState::view->flag_display_curvature))
        {
          viewer.data(GlobalState::id_curvature).set_visible(GlobalState::view->flag_display_curvature);
          GlobalState::activate_update();
        }

        if (ImGui::InputInt("frequency curvature", &(GlobalState::view->point_to_curvature)))
        {
          GlobalState::view->point_to_curvature = GlobalState::view->point_to_curvature >= 0 ? GlobalState::view->point_to_curvature : 0;
          GlobalState::activate_update();
        }

        double p_step = 0.01;
        double p_step_fast = 0.1;
        if (ImGui::InputScalar("comb scale", ImGuiDataType_Double, &GlobalState::view->scale_curvature, &p_step, &p_step_fast, "%.2lf"))
        {
          modification();
        }

        if (ImGui::Checkbox("heightmap color", &GlobalState::view->use_height_curvature_color))
        {
          GlobalState::activate_update();
        }
      }
    }
  };
  // // Draw additional windows
  menu.callback_draw_custom_window = [&]()
  {
    //   Define next window position + size
    ImGui::SetNextWindowPos(ImVec2((viewer.core().viewport(2) / 2 - 340.f) * menu.menu_scaling(), 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320.f * menu.menu_scaling(), 5 * viewer.core().viewport(3) / 12), ImGuiCond_FirstUseEver);
    ImGui::Begin(
        "Model", nullptr,
        ImGuiWindowFlags_NoSavedSettings);

    if (ImGui::CollapsingHeader("Default Models", ImGuiTreeNodeFlags_DefaultOpen))
    {
      ImGui::Combo("model type", (int *)(&Standard_Models::model_type), "Line\0Circle\0Star\0Heart\0Random\0Polygon Random\0Convex Hull Random\0Box Random\0Compare1\0Compare2\0Omega\0OmegaSimple\0Input\0\0");
      if (ImGui::InputInt("nb points", &(Standard_Models::number_points)))
      {
        Standard_Models::number_points = Standard_Models::number_points <= 1 ? 1 : Standard_Models::number_points;
      };

      double p_step = 0.1;
      double p_step_fast = 1.;
      ImGui::InputScalar("scale", ImGuiDataType_Double, &Standard_Models::scale_model, &p_step, &p_step_fast, "%.2lf");
      if (ImGui::Button("get model", ImVec2(-1.0, 0)))
      {
        GlobalState::controller->clear_selected();
        Eigen::MatrixXd Points;
        Standard_Models::get_standard_model(Points, Standard_Models::scale_model, Standard_Models::number_points);
        GlobalState::spline->set_control_points(Points);
        if (Points.rows() > 1)
          viewer.core().align_camera_center(Points);
        viewer.core().camera_translation = Eigen::Vector3f::Zero();
        viewer.core().camera_zoom = 1.f;
        GlobalState::activate_update();
      }
    }

    if (ImGui::CollapsingHeader("Control Points", 0))
    {
      if (ImGui::Button("erase", ImVec2(-1.0, 0)))
      {
        GlobalState::spline->hardreset();
        GlobalState::controller->clear_selected();
        GlobalState::activate_update();
      }
      if (ImGui::Button("print", ImVec2(-1.0, 0)))
      {
        Eigen::MatrixXd Points, Tangents;
        Eigen::VectorXd Curvatures;
        GlobalState::spline->get_control_points(Points);
        GlobalState::spline->get_control_tangents(Tangents);
        GlobalState::spline->get_control_curvatures(Curvatures);
        M_Log(Points.block(0, 0, Points.rows(), 2), "Current Control Points");
        M_Log(Tangents.block(0, 0, Points.rows(), 2), "Current Control Tangents");
        M_Log(Curvatures, "Current Control Curvatures");
      }
    }

    if (ImGui::CollapsingHeader("Splines", ImGuiTreeNodeFlags_DefaultOpen))
    {
      if (ImGui::Combo("Spline Type", (int *)(&GlobalState::spline->spline_type), "Clotho_3Arcs\0Clotho_Line\0\0"))
      {
        modification();
      }
      if (ImGui::Checkbox("cycle", &GlobalState::spline->cycle))
      {
        modification();
      }
      if (ImGui::Combo("Increase Type", (int *)(&increase_function), "Linear\0MaxLinear\0\0"))
      {
        modification();
      }

      if (ImGui::Checkbox("G1-Curvature", &GlobalState::spline->use_g1_curvature))
      {
        modification();
      }

      if (ImGui::Checkbox("refine", &GlobalState::spline->refine))
      {
        modification();
      }
      if (GlobalState::spline->spline_type == Spline_Type::Curve_Line_Clothoid)
      {

        if (ImGui::Checkbox("3-arcs correction", &GlobalState::spline->use_correction_CLC))
        {
          modification();
        }
        if (ImGui::Checkbox("intersection", &GlobalState::spline->intersection_detection))
        {
          modification();
        }
        if (GlobalState::spline->intersection_detection)
        {
          if (ImGui::InputInt("neighbors_intersection", &GlobalState::spline->neighbors_intersection))
          {
            if (GlobalState::spline->neighbors_intersection < -1)
            {
              GlobalState::spline->neighbors_intersection = -1;
            }
            modification();
          }
        }
      }
    }

    // Controller options
    if (ImGui::CollapsingHeader("Controller Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
      ImGui::Checkbox("multi-selection", &Controller::multi_selection);
      ImGui::Checkbox("rotation", &Controller::rotation);
      if (ImGui::Button("clear selection", ImVec2(-1.0, 0.)))
      {
        Controller::clear_selected();
        GlobalState::activate_update();
      }
      if (ImGui::Button("duplicate", ImVec2(-1.0, 0.)))
      {
        GlobalState::controller->duplicate_selected();
        GlobalState::activate_update();
      }
      if (GlobalState::controller->get_selected_points().size() == 1)
      {
        const int index_selected = *(GlobalState::controller->get_selected_points().begin());
        if (ImGui::CollapsingHeader("One point selected", ImGuiTreeNodeFlags_DefaultOpen))
        {
          Eigen::RowVector3d selected_point;
          GlobalState::spline->get_control_point(index_selected, selected_point);
          double p_step = 0.01, p_fast = 0.1;
          if (ImGui::InputDouble("x-coord", &selected_point(0), p_step, p_fast, "%.2f"))
          {
            GlobalState::spline->move_control_point(selected_point.transpose(), index_selected);
          }
          if (ImGui::InputDouble("y-coord", &selected_point(1), p_step, p_fast, "%.2f"))
          {
            GlobalState::spline->move_control_point(selected_point.transpose(), index_selected);
          }
          if (ImGui::Button("Cons. modifier", ImVec2(-1.0, 0.)))
          {
            GlobalState::controller->load_constraints(index_selected);
            GlobalState::modify_constraints_bool();
            GlobalState::activate_update();
          }
          ImGui::Checkbox("Only Curvature", &Controller::constraints_just_curvature);
          if (ImGui::Button("Validate Constraints", ImVec2(-1.0, 0.)))
          {
            if (GlobalState::must_constraints_input())
            {
              GlobalState::spline->controlPoints->constraints->add_control_curvature(index_selected, GlobalState::controller->constraint_curvature);
              if (!GlobalState::controller->constraints_just_curvature)
              {
                GlobalState::spline->controlPoints->constraints->add_control_tangent(index_selected, GlobalState::controller->constraint_tangent);
              }
              GlobalState::deactivate_constraints_input();
              modification();
            }
          }
          if (ImGui::Button("Remove All Constraints", ImVec2(-1.0, 0.)))
          {
            Eigen::MatrixXd control_points;
            GlobalState::spline->controlPoints->get_points(control_points);
            GlobalState::spline->controlPoints->constraints->reset(control_points.rows());
            modification();
          }
          if (ImGui::Button("Remove Tangent", ImVec2(-1.0, 0.)))
          {
            GlobalState::spline->controlPoints->constraints->remove_control_tangent(index_selected);
            modification();
          }
          if (ImGui::Button("Remove Curvature", ImVec2(-1.0, 0.)))
          {
            GlobalState::spline->controlPoints->constraints->remove_control_curvature(index_selected);
            modification();
          }
          if (ImGui::Button("Opposite Curvature", ImVec2(-1.0, 0.)))
          {
            GlobalState::controller->constraint_curvature *= -1;
            GlobalState::activate_update();
          }
        }
      }
    }
    ImGui::End();
  };
}