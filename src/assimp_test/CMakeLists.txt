AUX_SOURCE_DIRECTORY(. App_LIST)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})

foreach( current_app ${App_LIST} )
    # I used a simple string replace, to cut off .cpp.
    get_filename_component(app_name  ${current_app}  NAME_WE)
    add_executable( ${app_name}  ${current_app})
    message("Now name is : " ${app_name})
    # Make sure YourLib is linked to each app
    target_link_libraries(${app_name}  ${PROJECT_NAME}_3d ${OpenCV_LIBRARIES} ${GLUT_LIBRARIES})
    
endforeach( current_app ${App_LIST} )

