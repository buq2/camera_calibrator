find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Catch2 REQUIRED)

add_library(calibrator
    calibrator.cpp calibrator.hh
    data_generator.cpp data_generator.hh
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
