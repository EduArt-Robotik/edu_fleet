ament_add_gtest(test_kalman_filter
  unittest_observation_matrix.cpp
  unittest_attribute_vector.cpp
  unittest_sensor_model.cpp
  unittest_kalman_filter_model_mecanum.cpp
  unittest_extended_kalman_filter.cpp
)
target_include_directories(test_kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_kalman_filter
  eduart-fleet-kalman-filter
  eduart-fleet-sensor-model
)
