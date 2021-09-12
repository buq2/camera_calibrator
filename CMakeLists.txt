cmake_minimum_required(VERSION 3.5)
project(calibrator VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

include(cmake/conan_config.cmake)
include(cmake/output_paths.cmake)

# Generate compile_commands.json for vscode use when Ninja is used
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(src)
