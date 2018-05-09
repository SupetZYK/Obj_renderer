#AUX_SOURCE_DIRECTORY(. App_LIST)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#foreach( current_app ${App_LIST} )
#    # I used a simple string replace, to cut off .cpp.
#    get_filename_component(app_name  ${current_app}  NAME_WE)
#    add_executable( ${app_name}  ${current_app})
#    message("Now name is : " ${app_name})
#    # Make sure YourLib is linked to each app
#    target_link_libraries(${app_name}  ${PROJECT_NAME}_3d ${OpenCV_LIBRARIES} ${EXTRA_LIBRARIES} )
#    # install(TARGETS ${app_name}
#    #     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#endforeach( current_app ${App_LIST} )


add_executable(project_render_test project_render_test.cpp)
target_link_libraries(project_render_test ${OpenCV_LIBRARIES} ${PROJECT_NAME}_3d)

#add_executable(train_test train_test.cpp)
#target_link_libraries(train_test ${OpenCV_LIBRARIES})

add_executable(view_generate view_generate.cpp)
target_link_libraries(view_generate ${OpenCV_LIBRARIES} ${PROJECT_NAME}_3d)

add_executable(simple_opengl_test simple_opengl_test.cpp)
target_link_libraries(simple_opengl_test ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${ASSIMP_LIBRARIES})

######################
# ppf trainner, need pcl support and zyk_ppf
######################
find_package(PCL 1.8 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
set(zyk_ppf_lib /home/zyk/git/qt/zyk_ppf/build-src-Desktop-Release/libzyk_ppf.so)
set(zyk_ppf_include_dir /home/zyk/git/zyk_ppf/src)
include_directories(${zyk_ppf_include_dir})
add_executable(view_based_ppf_trainner view_based_ppf_trainner.cpp)
target_link_libraries(view_based_ppf_trainner ${OpenCV_LIBRARIES} ${PROJECT_NAME}_3d ${PCL_LIBRARIES} ${zyk_ppf_lib})
