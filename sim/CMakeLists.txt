cmake_minimum_required(VERSION 2.8.6)
project(sim)

add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)
SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

find_package(TinyXML REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost 1.66.0 COMPONENTS filesystem python3 numpy3)

add_compile_options(-DHAVE_CSTDDEF)
add_definitions(-DOCTOCON_DIR="${CMAKE_HOME_DIRECTORY}")
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${TinyXML_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

link_directories(./fem/)

include(FindOpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

file(GLOB srcs "*.h" "*.cpp")

# find_package(DART REQUIRED COMPONENTS utils-urdf gui optimizer-ipopt CONFIG)
# add_compile_options(-DHAVE_CSTDDEF)
# include_directories(${DART_INCLUDE_DIRS})

add_library(sim ${srcs})
target_link_libraries(sim ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} #[[${DART_LIBRARIES}]] ${TinyXML_LIBRARIES} fem)

add_subdirectory(fem)
