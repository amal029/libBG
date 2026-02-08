cmake_minimum_required(VERSION 4.0) # CMake version check
set(CMAKE_C_COMPILER cc)
set(CMAKE_CXX_COMPILER c++)
project(libBG)
set(CMAKE_CXX_STANDARD 20)

# Include the FindPkgConfig module
find_package(PkgConfig REQUIRED)

# Get package ginac
# pkg_check_modules(GINAC REQUIRED ginac)

# For lsp language server
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add ex1.cpp file of the project root directory as a source file
set(SOURCE_FILES ../src/ex1.cpp)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(test ${SOURCE_FILES})

# The include directory
# target_include_directories(test PRIVATE includes ${GINAC_INCLUDE_DIRS})
target_include_directories(test PRIVATE includes)

target_compile_options(test PRIVATE -O3 -Wall -Werror -Wpedantic -Wextra
			    -Wno-unused-parameter -Wno-unused-function)
# target_link_directories(test PRIVATE ${GINAC_LIBRARY_DIRS})
# target_link_libraries(test PRIVATE ${GINAC_LIBRARIES})


