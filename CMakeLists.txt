cmake_minimum_required(VERSION 2.8.9)
project(particle_filter)

# Set C++ 11
add_definitions(-std=c++11)

# Include the headers
include_directories(include)

# Find Opencv
find_package( OpenCV REQUIRED )

# To add individual files manually:
# set(SOURCES <filepath_1> <filepath_2>)

# To add all the files in a loop
file(GLOB SOURCES "src/*.cpp")

# Generate the shared library from the sources
# add_library(particle_filter SHARED ${SOURCES})

# Install the library
# install(TARGETS particle_filter DESTINATION /usr/lib)

# Make the executable
add_executable(main src/main.cxx)
add_executable(motion_model_test src/motion_model_test.cpp src/motion_model_odometry.cpp)

# Link the libraries
target_link_libraries(main ${OpenCV_LIBS})
# target_link_libraries(main particle_filter)
