cmake_minimum_required(VERSION 3.29)
project(Raster)

set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(Raster main.cpp
        Triangle.cpp
        Triangle.h
        Texture.cpp
        Texture.h
        Model.cpp
        Model.h
        OBJ_Loader.h
        Shader.cpp
        Shader.h
        rasterizer.cpp
        rasterizer.h
        Fragment.h
        Camera.cpp
        Camera.h
        BVH.cpp
        BVH.h
        BoundingBox.cpp
        BoundingBox.h
        Ray.cpp
        Ray.h)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})