# add_executable(go2_trajectory_follow go2_trajectory_follow.cpp)
# target_link_libraries(go2_trajectory_follow unitree_sdk2)

add_executable(go2_sport_client go2_sport_client.cpp)
target_link_libraries(go2_sport_client unitree_sdk2)

add_executable(go2_low_level go2_low_level.cpp)
target_link_libraries(go2_low_level unitree_sdk2)

add_executable(go2_stand_example go2_stand_example.cpp)
target_link_libraries(go2_stand_example unitree_sdk2)


add_executable(go2_robot_state_client go2_robot_state_client.cpp)
target_link_libraries(go2_robot_state_client unitree_sdk2)

add_executable(go2_video_client go2_video_client.cpp)
target_link_libraries(go2_video_client unitree_sdk2)

add_executable(go2_vui_client go2_vui_client.cpp)
target_link_libraries(go2_vui_client unitree_sdk2)

add_executable(go2_cmd_vel go2_cmd_vel.cpp)
target_link_libraries(go2_cmd_vel unitree_sdk2)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(classic_gait_initializer_test
    classic_gait_initializer_test.cpp
  )
  if(TARGET classic_gait_initializer_test)
    target_link_libraries(classic_gait_initializer_test ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(classic_gait_lock_test
    classic_gait_lock_test.cpp
  )
  if(TARGET classic_gait_lock_test)
    target_link_libraries(classic_gait_lock_test ${catkin_LIBRARIES})
  endif()
endif()

# 将程序安装到 Catkin 的目标位置
install(TARGETS go2_cmd_vel
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
