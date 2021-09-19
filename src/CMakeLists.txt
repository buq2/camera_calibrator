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


add_executable(test_calibrator
    test_calibrator.cpp)
target_link_libraries(test_calibrator
    Catch2::Catch2
    calibrator
    )


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

ADD_EXECUTABLE(test_gui
    main_gui.cpp
    )

TARGET_LINK_LIBRARIES(test_gui
    gui
    )
