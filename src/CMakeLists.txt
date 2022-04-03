find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Catch2 REQUIRED)

add_library(calibrator
    calibrator.cpp calibrator.hh
    extrinsics_calibrator.cpp extrinsics_calibrator.hh
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
    log_gui)


FIND_PACKAGE(SDL2 REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(imgui REQUIRED)
FIND_PACKAGE(implot REQUIRED)
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
    implot::implot
    opencv::core  
    opencv::imgproc
    opencv::highgui
    )
TARGET_INCLUDE_DIRECTORIES(gui
    PUBLIC ${IMGUI_BINDINGS_DIR}
    )

add_library(log_gui
    log_image.cpp log_plot.cpp
    )
target_link_libraries(log_gui
    gui)

ADD_EXECUTABLE(test_gui
    main_gui.cpp
    )

TARGET_LINK_LIBRARIES(test_gui
    gui
    log_gui
    corners
    )


add_executable(test_calibrator
    test_calibrator.cpp)
target_link_libraries(test_calibrator
    Catch2::Catch2
    calibrator
    )

add_executable(test_extrinsics_calibrator
    test_extrinsics_calibrator.cpp)
target_link_libraries(test_extrinsics_calibrator
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
catch_discover_tests(test_extrinsics_calibrator)
catch_discover_tests(test_geometry)
