cmake_minimum_required(VERSION 3.16)
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_BUILD_TYPE Release)

project(smooth-interpolating-monocurvature)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"    ON) 
option(LIBIGL_USE_STATIC_LIBRARY     "Use libIGL as static librarie" OFF)
include(libigl)
add_compile_definitions(IGL_VIEWER_VIEWER_QUIET)

# Libhedra
include_directories(../extern/libhedra/include)

# Clothoids 
include_directories(../extern/clothoids/src) 
include_directories(../extern/clothoids/lib3rd/include)
include_directories(../extern/clothoids/lib/include)
include_directories(../extern/clothoids/submodules/Utils/src/Utils)
FILE(GLOB CLOTHOIDS_FILES ../extern/clothoids/src/*.hh ../extern/clothoids/src/*.cc ../extern/clothoids/src/Clothoids/*.hxx ../extern/clothoids/src/lib3rd/include/*)
FILE(GLOB SUB_CLOTHOIDS_FILES ../extern/clothoids/submodules/Utils/src/*.cc ../extern/clothoids/submodules/Utils/src/*.hh ../extern/clothoids/submodules/Utils/src/Utils/* ../extern/clothoids/submodules/quarticRootsFlocke/src/*)
 
# Add your project files 
FILE(GLOB MODEL_FILES src/model/*.h src/model/*.cpp)  
FILE(GLOB VIEW_FILES src/view/*.h src/view/*.cpp)
FILE(GLOB CONTROLLER_FILES src/controller/*.h src/controller/*.cpp) 
FILE(GLOB UTILS_FILES src/utils/*.h src/utils/*.cpp) 
FILE(GLOB SRC_FILES src/*.cpp src/*.h)
add_executable(${PROJECT_NAME} ${SRC_FILES} ${UTILS_FILES} ${MODEL_FILES} ${VIEW_FILES} ${CONTROLLER_FILES} ${CLOTHOIDS_FILES} ${SUB_CLOTHOIDS_FILES}) 
target_link_libraries(${PROJECT_NAME} PUBLIC igl::core igl::opengl_glfw igl::opengl_glfw_imgui) 