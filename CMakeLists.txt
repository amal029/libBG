cmake_minimum_required(VERSION 4.0) # CMake version check
set(CMAKE_C_COMPILER cc)
set(CMAKE_CXX_COMPILER c++)
project(libBG)
# Need C++23
set(CMAKE_CXX_STANDARD 23)

# Include the FindPkgConfig module
# find_package(PkgConfig REQUIRED)

# Get package ginac
# pkg_check_modules(GINAC REQUIRED ginac)

# For lsp language server
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The include directory
set(INCLUDES src/includes)

# The compiler options
set (CXX_FLAGS -O3 -Wall -Werror -Wpedantic -Wextra -Wno-unused-parameter -Wno-unused-function)

# Add ex1.cpp file of the project root directory as a source file
# set(SOURCE_FILES ../test/test1.cpp)

#------------------------Test executables from here ------------------
# Add executable target with source files listed in SOURCE_FILES variable
add_executable(test1 ../test/test1.cpp)

# The include directory
# target_include_directories(test1 PRIVATE ${INCLUDES} ${GINAC_INCLUDE_DIRS})
target_include_directories(test1 PRIVATE ${INCLUDES})

target_compile_options(test1 PRIVATE ${CXX_FLAGS})
# target_link_directories(test1 PRIVATE ${GINAC_LIBRARY_DIRS})
# target_link_libraries(test1 PRIVATE ${GINAC_LIBRARIES})

#--------------------------Test 2--------------------------------------
# The second test
# Add executable target with source files listed in SOURCE_FILES variable
add_executable(test2 ../test/test2.cpp)

# The include directory
target_include_directories(test2 PRIVATE ${INCLUDES})

target_compile_options(test2 PRIVATE ${CXX_FLAGS})
# target_link_directories(test2 PRIVATE ${GINAC_LIBRARY_DIRS})
# target_link_libraries(test2 PRIVATE ${GINAC_LIBRARIES})

#----------------------- Example 1---------------------
# Add executable target with source files listed in SOURCE_FILES variable
add_executable(ex1 ../test/ex1.cpp)

# The include directory
# target_include_directories(ex1 PRIVATE ${INCLUDES} ${GINAC_INCLUDE_DIRS})
target_include_directories(ex1 PRIVATE ${INCLUDES})

target_compile_options(ex1 PRIVATE ${CXX_FLAGS})
# target_link_directories(ex1 PRIVATE ${GINAC_LIBRARY_DIRS})
# target_link_libraries(ex1 PRIVATE ${GINAC_LIBRARIES})

# ------------Expression test-------------------------
add_executable(testexp ../test/testexp.cpp)

# The include directory
# target_include_directories(testexp PRIVATE ${INCLUDES} ${GINAC_INCLUDE_DIRS})
target_include_directories(testexp PRIVATE ${INCLUDES})
target_compile_options(testexp PRIVATE ${CXX_FLAGS})

