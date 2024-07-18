ament_add_gtest(test_observation_matrix
  unittest_observation_matrix.cpp
)
target_include_directories(test_observation_matrix PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_observation_matrix
  eduart-fleet-kalman-filter
)
