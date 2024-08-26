cmake_minimum_required(VERSION 3.28)
project(Rasterizer)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(EIGEN3_INCLUDE_DIR)
include_directories(/usr/local/include)

add_executable(Rasterizer main.cpp rasterizer.hpp Triangle.hpp Texture.hpp Shader.hpp OBJ_Loader.h rasterizer.cpp Triangle.cpp Texture.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)