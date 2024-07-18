add_library(eduart-fleet-kalman-filter
  observation_matrix_handler.cpp
)
target_link_libraries(eduart-fleet-kalman-filter
  Eigen3::Eigen
)
target_compile_features(eduart-fleet-kalman-filter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-fleet-kalman-filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ament_target_dependencies(eduart-fleet-kalman-filter
#   "rclcpp"
# )
