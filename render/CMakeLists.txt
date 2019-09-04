cmake_minimum_required(VERSION 2.8.6)
project(render)

add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)
SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)

link_directories(../sim/)
include_directories(../sim/)
link_directories(../sim/fem/)
include_directories(../sim/fem/)

find_package(Boost 1.66.0 COMPONENTS python3 numpy3 filesystem)
find_package(PythonLibs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include(FindOpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
add_compile_options(-DHAVE_CSTDDEF)
include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

file(GLOB srcs "*.h" "*.cpp")

add_executable(render ${srcs})
target_link_libraries(render ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${Boost_LIBRARIES} 
	${PYTHON_LIBRARIES} sim /usr/lib/x86_64-linux-gnu/libtinyxml.a fem)