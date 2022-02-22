find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Catch2 REQUIRED)

add_library(calibrator
    calibrator.cpp calibrator.hh
    data_generator.cpp data_generator.hh
    geometry.cpp geometry.hh
    )
target_link_libraries(calibrator
    PUBLIC 
    opencv::core  
    opencv::imgproc
    opencv::calib3d
    Ceres::ceres
    Eigen3::Eigen
    )

add_library(corners
    corners.cpp corners.hh
    )
target_link_libraries(corners
    PUBLIC
    opencv::core
    opencv::imgproc
    log_image)


FIND_PACKAGE(SDL2 REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(imgui REQUIRED)
SET(IMGUI_BINDINGS_DIR ${imgui_RES_DIRS}/bindings/)

add_library(gui
    gui.cpp gui.hh
    scene.cpp scene.hh
    ${IMGUI_BINDINGS_DIR}/imgui_impl_sdl.cpp
    ${IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp
    )
target_link_libraries(gui
    SDL2::SDL2main
    SDL2::SDL2
    GLEW::GLEW
    imgui::imgui
    opencv::core  
    opencv::imgproc
    opencv::highgui
    )
TARGET_INCLUDE_DIRECTORIES(gui
    PUBLIC ${IMGUI_BINDINGS_DIR}
    )

add_library(log_image
    log_image.cpp log_image.cpp)
target_link_libraries(log_image
    gui)

ADD_EXECUTABLE(test_gui
    main_gui.cpp
    )

TARGET_LINK_LIBRARIES(test_gui
    gui
    log_image
    corners
    )


add_executable(test_calibrator
    test_calibrator.cpp)
target_link_libraries(test_calibrator
    Catch2::Catch2
    calibrator
    )

add_executable(test_geometry
    test_geometry.cpp)
target_link_libraries(test_geometry
    Catch2::Catch2
    calibrator
    )

add_executable(test_common
    test_common.cpp)

target_link_libraries(test_common
    Catch2::Catch2
    )

include(CTest)
include(Catch)
catch_discover_tests(test_calibrator)
catch_discover_tests(test_geometry)
