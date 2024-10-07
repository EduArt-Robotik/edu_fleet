add_library(eduart-fleet-transform
  geometry.cpp
)
target_link_libraries(eduart-fleet-transform
  Eigen3::Eigen
)
target_compile_features(eduart-fleet-transform PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-fleet-transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(eduart-fleet-transform
  "geometry_msgs"
  "sensor_msgs"
)
