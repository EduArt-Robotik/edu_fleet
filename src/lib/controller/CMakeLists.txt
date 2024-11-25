add_library(eduart-fleet-controller
  pid.cpp
)
target_link_libraries(eduart-fleet-controller

)
target_compile_features(eduart-fleet-controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-fleet-controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ament_target_dependencies(eduart-fleet-controller

# )
