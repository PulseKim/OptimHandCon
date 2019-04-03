cmake_minimum_required(VERSION 2.8.6)
project(finger)

add_compile_options(-fPIC)
add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)
SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)

find_package(DART REQUIRED COMPONENTS gui collision-bullet optimizer-ipopt CONFIG)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_compile_options(-DHAVE_CSTDDEF)
include_directories(${DART_INCLUDE_DIRS})
message(${DART_LIBRARIES})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
file(GLOB srcs "*.h" "*.cpp")

add_executable(finger ${srcs})
target_link_libraries(finger ${DART_LIBRARIES} GL GLU glut)
