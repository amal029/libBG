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

# The third-party dir
set(THIRD_PARTY_DIR src/thirdparty)

# The third party cpp file
file(GLOB_RECURSE THIRD_PARTY_SRCS ${THIRD_PARTY_DIR}/*.cpp)
message(${THIRD_PARTY_SRCS})

# For lsp language server
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The include directory
set(INCLUDES src/includes ${THIRD_PARTY_DIR})

# The compiler options
set (CXX_FLAGS -O3 -Wall -Werror -Wpedantic -Wextra -Wno-unused-parameter -Wno-unused-function)

# Add ex1.cpp file of the project root directory as a source file
# set(SOURCE_FILES ../test/test1.cpp)

#------------------------Test executables from here (Test 1)------------------
# Add executable target with source files listed in SOURCE_FILES variable
add_executable(RLClowpass ../test/RLClowpass.cpp ${THIRD_PARTY_SRCS})

# The include directory
# target_include_directories(RLClowpass PRIVATE ${INCLUDES} ${GINAC_INCLUDE_DIRS})
target_include_directories(RLClowpass PRIVATE ${INCLUDES} ${THIRD_PARTY_DIR})

target_compile_options(RLClowpass PRIVATE ${CXX_FLAGS})
# target_link_directories(test1 PRIVATE ${GINAC_LIBRARY_DIRS})
# target_link_libraries(test1 PRIVATE ${GINAC_LIBRARIES})

#-------------Test executables from here (Compensator controller)------------------
# Add executable target with source files listed in SOURCE_FILES variable
add_executable(ccontrol ../test/massvelCompensator.cpp ${THIRD_PARTY_SRCS})

# The include directory
target_include_directories(ccontrol PRIVATE ${INCLUDES} ${THIRD_PARTY_DIR})
target_compile_options(ccontrol PRIVATE ${CXX_FLAGS})


#------------------------Test executables from here (P controller)------------------
# Add executable target with source files listed in SOURCE_FILES variable
add_executable(mvpcontrol ../test/massvelocityPcontrol.cpp ${THIRD_PARTY_SRCS})

# The include directory
target_include_directories(mvpcontrol PRIVATE ${INCLUDES} ${THIRD_PARTY_DIR})

target_compile_options(mvpcontrol PRIVATE ${CXX_FLAGS})

#--------------------------Test 2--------------------------------------
# The second test
# Add executable target with source files listed in SOURCE_FILES variable
add_executable(hoistingDevice ../test/hoistingDevice.cpp ${THIRD_PARTY_SRCS})

# The include directory
target_include_directories(hoistingDevice PRIVATE ${INCLUDES} ${THIRD_PARTY_DIR})

target_compile_options(hoistingDevice PRIVATE ${CXX_FLAGS})
# target_link_directories(hoistingDevice PRIVATE ${GINAC_LIBRARY_DIRS})
# target_link_libraries(hoistingDevice PRIVATE ${GINAC_LIBRARIES})

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

