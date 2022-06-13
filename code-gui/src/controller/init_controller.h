#pragma once

#include "../global.h"

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include "controller.h"
#include "../model/init_model.h"
#include "../model/clothoid_line.h"

void initialize_menu(igl::opengl::glfw::imgui::ImGuiMenu &menu);