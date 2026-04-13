cmake_minimum_required(VERSION 3.27.1)

project(rasteriser)

add_subdirectory(3rdParty)

include_directories(3rdParty/lodepng)

add_executable(rasteriser
    rasteriser.cpp
    Shading.hpp
    Image.hpp
    Light.hpp
    LinAlg.hpp
    Mesh.hpp
    )


target_link_libraries(rasteriser 
    lodepng
    )