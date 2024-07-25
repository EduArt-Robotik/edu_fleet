#include <gtest/gtest.h>

#include <edu_fleet/kalman_filter/attribute.hpp>
#include <edu_fleet/kalman_filter/attribute_vector_plotter.hpp>
#include <edu_fleet/kalman_filter/filter_model_mecanum.hpp>

using eduart::fleet::kalman_filter::Data;
using eduart::fleet::kalman_filter::FilterModelMecanum;
using eduart::fleet::kalman_filter::AttributeVectorPlotter;

class UnitTestKalmanFilterMecanum : public testing::Test
{
public:
  void SetUp() override {
    
  }
  void TearDown() override {

  }

protected:
  inline static constexpr std::size_t model_dimension = FilterModelMecanum::attribute_vector::size();
  FilterModelMecanum model = FilterModelMecanum({});

  FilterModelMecanum::attribute_vector state;
  Eigen::Matrix<Data, model_dimension, model_dimension> covariance = Eigen::Matrix<Data, model_dimension, model_dimension>::Zero();
};

TEST(kalman_filter_model_mecanum, instantiation)
{
  constexpr std::size_t model_dimension = 8;
  FilterModelMecanum model({});
  FilterModelMecanum::attribute_vector state;

  EXPECT_EQ(model.rows(), model_dimension);
  EXPECT_EQ(model.cols(), model_dimension);

  state.yaw() = -3.14;
  std::cout << "prediction matrix:\n" << model.getPredictionMatrix(state, 0.1) << std::endl;
  state.yaw() = 3.14;
  std::cout << "prediction matrix:\n" << model.getPredictionMatrix(state, 0.1) << std::endl;
}

TEST_F(UnitTestKalmanFilterMecanum, prediction_x_based_on_acceleration)
{
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter("prediction-x-based-on-acceleration.csv");

  constexpr Data dt = 0.01; // 10ms == 100Hz
  constexpr Data stamp_end = 10.0; // 10s

  // plot initial state
  plotter.printState(state, 0.0);
  // set constant acceleration in x direction
  state.acceleration_x() = 1.0;

  for (Data stamp = 0.0; stamp < stamp_end; stamp += dt) {
    // do: state = F * state
    state.set(model.getPredictionMatrix(state, dt) * state.get());
    plotter.printState(state, stamp);
  }
}

TEST_F(UnitTestKalmanFilterMecanum, prediction_y_based_on_acceleration)
{
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter("prediction-y-based-on-acceleration.csv");

  constexpr Data dt = 0.01; // 10ms == 100Hz
  constexpr Data stamp_end = 10.0; // 10s

  // plot initial state
  plotter.printState(state, 0.0);
  // set constant acceleration in x direction
  state.acceleration_y() = 1.0;

  for (Data stamp = 0.0; stamp < stamp_end; stamp += dt) {
    // do: state = F * state
    state.set(model.getPredictionMatrix(state, dt) * state.get());
    plotter.printState(state, stamp);
  }
}

TEST_F(UnitTestKalmanFilterMecanum, prediction_x_based_on_velocity)
{
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter("prediction-x-based-on-velocity.csv");

  constexpr Data dt = 0.01; // 10ms == 100Hz
  constexpr Data stamp_phase1_end = 5.0;  //  5s
  constexpr Data stamp_phase2_end = 10.0; // 10s

  // plot initial state
  plotter.printState(state, 0.0);
  // set constant velocity in x direction
  state.velocity_x() = 1.0;

  for (Data stamp = 0.0; stamp < stamp_phase1_end; stamp += dt) {
    // do: state = F * state
    state.set(model.getPredictionMatrix(state, dt) * state.get());
    plotter.printState(state, stamp);
  }

  // change velocity
  state.velocity_x() = 5.0;

  for (Data stamp = stamp_phase1_end; stamp < stamp_phase2_end; stamp += dt) {
    // do: state = F * state
    state.set(model.getPredictionMatrix(state, dt) * state.get());
    plotter.printState(state, stamp);    
  }
}

TEST_F(UnitTestKalmanFilterMecanum, prediction_y_based_on_velocity)
{
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter("prediction-y-based-on-velocity.csv");

  constexpr Data dt = 0.01; // 10ms == 100Hz
  constexpr Data stamp_phase1_end = 5.0;  //  5s
  constexpr Data stamp_phase2_end = 10.0; // 10s

  // plot initial state
  plotter.printState(state, 0.0);
  // set constant velocity in x direction
  state.velocity_y() = 1.0;

  for (Data stamp = 0.0; stamp < stamp_phase1_end; stamp += dt) {
    // do: state = F * state
    state.set(model.getPredictionMatrix(state, dt) * state.get());
    plotter.printState(state, stamp);
  }

  // change velocity
  state.velocity_y() = 5.0;

  for (Data stamp = stamp_phase1_end; stamp < stamp_phase2_end; stamp += dt) {
    // do: state = F * state
    state.set(model.getPredictionMatrix(state, dt) * state.get());
    plotter.printState(state, stamp);    
  }
}

TEST_F(UnitTestKalmanFilterMecanum, prediction_pos_based_on_velocity_and_yaw_rate)
{
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter(
    "prediction-pose-based-on-velocity-and-yaw-rate.csv");

  constexpr Data dt = 0.01; // 10ms == 100Hz
  constexpr Data stamp_end = 10.0; // 10s
  constexpr Data velocity = 1.0; // 1m/s

  // plot initial state
  plotter.printState(state, 0.0);
  // set constant acceleration in x direction
  state.velocity_x() = velocity;
  state.yaw_rate() = (2.0 * M_PI) / stamp_end;

  for (Data stamp = 0.0; stamp < stamp_end; stamp += dt) {
    // do: state = F * state
    state.set(model.getPredictionMatrix(state, dt) * state.get());
    plotter.printState(state, stamp);
  }
}
