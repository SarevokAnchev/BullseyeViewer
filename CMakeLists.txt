cmake_minimum_required(VERSION 3.22)
project(BullseyeViewer)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES
        src/main.cpp
        src/bulleye/Bulleye.cpp
        src/bulleye/BulleyeSegment.cpp
        src/bulleye/BulleyeDisplay.cpp)

message(${CMAKE_BUILD_TYPE})

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(nlohmann_json_DIR "C:/Program Files (x86)/nlohmann_json/debug/lib/cmake/nlohmann_json")
    set(VTK_DIR "C:/Program Files (x86)/VTK/debug/lib/cmake/vtk-9.0")
    set(OpenCV_DIR "C:/Program Files (x86)/OpenCV/debug/x64/vc16/lib")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(nlohmann_json_DIR "C:/Program Files (x86)/nlohmann_json/release/lib/cmake/nlohmann_json")
    set(VTK_DIR "C:/Program Files (x86)/VTK/release/lib/cmake/vtk-9.0")
    set(OpenCV_DIR "C:/Program Files (x86)/OpenCV/release/x64/vc16/lib")
endif (CMAKE_BUILD_TYPE STREQUAL "Release")

find_package(VTK REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(BullseyeViewer ${SOURCES})

target_link_libraries(BullseyeViewer
        ${VTK_LIBRARIES}
        nlohmann_json::nlohmann_json)

vtk_module_autoinit(
        TARGETS BullseyeViewer
        MODULES ${VTK_LIBRARIES}
)
