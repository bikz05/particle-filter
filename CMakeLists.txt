cmake_minimum_required(VERSION 2.8.9)
project(particle_filter)

# Include the headers
include_directories(include)

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

# Link the libraries
# target_link_libraries(main particle_filter)
