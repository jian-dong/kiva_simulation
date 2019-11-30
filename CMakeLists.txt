cmake_minimum_required(VERSION 3.10)
project(kiva_simulation)

# C++ standards.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build only static libraries.
#set(BUILD_SHARED_LIBS OFF)

# Side note: use "VERBOSE=1 make" to show the actual make command.
set(CMAKE_CXX_FLAGS "-O2")
#set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-static")

set(EXECUTABLE_OUTPUT_PATH
        ${PROJECT_BINARY_DIR}/bin)

set(LIBRARY_OUTPUT_PATH
        ${PROJECT_BINARY_DIR}/lib)

add_subdirectory(src)
