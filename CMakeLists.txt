# ---------------------------------------------------------------------------#
# ------------------------------- CMake -------------------------------------#
# ---------------------------------------------------------------------------#

# minimum supported cmake version
cmake_minimum_required(VERSION 2.6)

# identify the project and programming language
project(cameracalibration CXX)

# compiler flags passed as arguments 
add_definitions(-Wall -Werror)

# set output directories for libraries and executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# find external libraries
find_package(OpenCV REQUIRED)

# specify the TARGET executable
add_executable(calibrate ${CMAKE_SOURCE_DIR}/src/calibrate.cc)

# link to external libraries
target_link_libraries(calibrate ${OpenCV_LIBS})

# ---------------------------------------------------------------------------#