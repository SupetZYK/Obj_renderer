# add an executable to render views to a file
add_executable(trainer_planar trainer_planar.cpp)
#SET_PROPERTY(TARGET trainer_planar PROPERTY DEBUG_POSTFIX ${ASSIMP_DEBUG_POSTFIX})
target_link_libraries(trainer_planar ${PROJECT_NAME}_2d ${OpenCV_LIBRARIES})
#SET_TARGET_PROPERTIES(trainer_planar PROPERTIES
#                                     OUTPUT_NAME trainer_planar
#)

# install(TARGETS trainer_planar
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
