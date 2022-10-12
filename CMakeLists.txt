cmake_minimum_required(VERSION 3.22)
project(BullseyeViewer)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES
        src/Bullseye.cpp
        src/BullseyeSegment.cpp
        src/BullseyeDisplay.cpp)

add_subdirectory(external/KDTree)

find_package(VTK REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(BullseyeViewer src/main.cpp ${SOURCES})
target_include_directories(BullseyeViewer PUBLIC include)

add_library(bullseye ${SOURCES})
target_include_directories(bullseye PUBLIC include)

target_link_libraries(BullseyeViewer
        PRIVATE ${VTK_LIBRARIES}
        PRIVATE nlohmann_json::nlohmann_json
        PRIVATE ${OpenCV_LIBS}
        PRIVATE kdtree)

target_link_libraries(bullseye
        PRIVATE ${VTK_LIBRARIES}
        PRIVATE nlohmann_json::nlohmann_json
        PRIVATE ${OpenCV_LIBS}
        PRIVATE kdtree)

vtk_module_autoinit(
        TARGETS BullseyeViewer bullseye
        MODULES ${VTK_LIBRARIES}
)

install(TARGETS bullseye LIBRARY DESTINATION lib)
