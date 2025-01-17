cmake_minimum_required(VERSION 3.28.1)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_OSX_ARCHITECTURES arm64)

project(slam VERSION 0.1.0 LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_TOOLCHAIN_FILE /Users/nolanschock/vcpkg/scripts/buildsystems/vcpkg.cmake)
include(/Users/nolanschock/vcpkg/scripts/buildsystems/vcpkg.cmake)
#"-I/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/c++/v1",
#"-I/Users/nolanschock/slam/include",

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(Pangolin CONFIG REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus CONFIG REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(G2O CONFIG REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

find_package(Ceres CONFIG REQUIRED)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        Sophus::Sophus
        ${Pangolin_LIBRARIES}# GL GLU GLEW glut
        Eigen3::Eigen
        g2o::core g2o::stuff g2o::types_sba
        Ceres::ceres
        )

enable_testing()

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
#add_subdirectory(test)
add_subdirectory(app)
