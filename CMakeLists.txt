cmake_minimum_required(VERSION 3.22)
project(BullseyeViewer)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES
        src/Bullseye.cpp
        src/BullseyeSegment.cpp
        src/BullseyeDisplay.cpp)

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    # Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT TARGET kdtree)
    add_subdirectory(external/KDTree)
endif()

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
