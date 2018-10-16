cmake_minimum_required(VERSION 2.8.0)
project(object_renderer)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED)
find_package(OpenCV 3 REQUIRED)


option(USE_GLUT "Use GLUT instead of OSMesa" OFF)

if (${USE_GLUT})
find_package(GLUT REQUIRED)
set(${PROJECT_NAME}_depends GLUT)
endif()

include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)
message("OpenCV FOUND IN" ${OpenCV_INCLUDE_DIRS})
add_subdirectory(src)

# install the include folder
install(FILES ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/renderer.h
	 ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/renderer2d.h
			   ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/renderer3d.h
			   ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/utils.h
   DESTINATION /usr/local/include/${PROJECT_NAME}
 )

# # add docs
# if (COMMAND ecto_sphinx)
# ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/ doc)
# endif()
