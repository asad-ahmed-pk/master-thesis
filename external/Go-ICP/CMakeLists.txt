# GO-ICP
# https://github.com/yangjiaolong/Go-ICP
# J. Yang, H. Li, Y. Jia, Go-ICP: Solving 3D Registration Efficiently and Globally Optimally, International Conference on Computer Vision (ICCV), 2013.

project(GoICP)
cmake_minimum_required(VERSION 3.10)

list(APPEND SOURCES
		src/jly_goicp.cpp
		src/jly_3ddt.cpp
		src/matrix.cpp
		src/ConfigMap.cpp
		src/StringTokenizer.cpp
)

add_library(GoICP STATIC ${SOURCES})
add_library(Jly::GoICP ALIAS GoICP)

# set include directories
target_include_directories(GoICP PUBLIC ${PROJECT_SOURCE_DIR}/include)