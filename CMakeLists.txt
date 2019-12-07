cmake_minimum_required(VERSION 3.10)
project(kiva_simulation)

# C++ standards.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build with llvm.
#SET (CMAKE_C_COMPILER             "/usr/bin/clang")
#SET (CMAKE_CXX_COMPILER           "/usr/bin/clang++")

# Build only static libraries.
#set(BUILD_SHARED_LIBS OFF)

# Side note: use "VERBOSE=1 make" to show the actual make command.
#set(CMAKE_CXX_FLAGS "-O2")
# When using -O2, some bug not exposed with -O0 may be exposed, so the code need
# to be more robust.
# -Wall : enable all warnings.
# -Werror: treat warning as error.
# Note, the two flags above effectively disables -O2.
set(CMAKE_CXX_FLAGS "-Wall -Wno-unused-function")
#set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-static")

set(EXECUTABLE_OUTPUT_PATH
        ${PROJECT_BINARY_DIR}/bin)

set(LIBRARY_OUTPUT_PATH
        ${PROJECT_BINARY_DIR}/lib)

add_subdirectory(src)