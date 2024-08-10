#include <gtest/gtest.h>

#include <edu_fleet/kalman_filter/attribute.hpp>
#include <edu_fleet/kalman_filter/attribute_pack.hpp>
#include <edu_fleet/kalman_filter/attribute_vector.hpp>
#include <edu_fleet/kalman_filter/observation_matrix_handler.hpp>
#include <edu_fleet/kalman_filter/extended_kalman_filter.hpp>
#include <edu_fleet/kalman_filter/filter_model_mecanum.hpp>
#include <edu_fleet/kalman_filter/attribute_vector_plotter.hpp>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <random>

using eduart::fleet::kalman_filter::Data;
using eduart::fleet::kalman_filter::ExtendedKalmanFilter;
using eduart::fleet::kalman_filter::Attribute;
using eduart::fleet::kalman_filter::AttributePack;
using eduart::fleet::kalman_filter::AttributeVector;
using eduart::fleet::kalman_filter::FilterModelMecanum;
using eduart::fleet::kalman_filter::FilterModelInterface;
using eduart::fleet::kalman_filter::AttributeVectorPlotter;
using eduart::fleet::kalman_filter::ObservationMatrixHandler;

// class ExtendedKalmanFilterTestObject : public ExtendedKalmanFilter<FilterModelMecanum::attribute_pack>
// {
// public:
//   ExtendedKalmanFilterTestObject(std::unique_ptr<FilterModelInterface> model)
//     : ExtendedKalmanFilter<FilterModelMecanum::attribute_pack>(std::move(model))
//   {

//   }
// };

TEST(extended_kalman_filter, predict_to_time_case1)
{
  // accelerate in x direction without system noise
  // using tested model mecanum to proof update method
  FilterModelMecanum::Parameter filter_parameter;

  filter_parameter.mode_type = FilterModelMecanum::Parameter::ModelType::PUSH_AND_ROTATE;
  filter_parameter.noise.acceleration = 0.0;
  filter_parameter.noise.yaw_rate = 0.0;

  auto filter_model = std::make_unique<FilterModelMecanum>(filter_parameter);
  ExtendedKalmanFilter<FilterModelMecanum::attribute_pack> extended_kalman_filter(std::move(filter_model));
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter_state(
    "extended_kalman_filter_predict_to_time_case1_state.csv");
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter_covariance(
    "extended_kalman_filter_predict_to_time_case1_cov.csv");

  rclcpp::Time stamp;
  rclcpp::Time stamp_end(10000000000); // 10s
  const rclcpp::Duration dt(0, 10000000); // 10ms
  FilterModelMecanum::attribute_vector init_state;
  Eigen::MatrixX<Data> init_covariance = Eigen::MatrixX<Data>::Identity(init_state.size(), init_state.size());

  init_state.acceleration_x() = 1.0;

  extended_kalman_filter.initialize(init_state.get(), init_covariance);
  plotter_state.printState(extended_kalman_filter.state(), 0);
  // plotter_covariance.printVarianceDiagonal(extended_kalman_filter.covariance(), 0);

  for (rclcpp::Time stamp(0); stamp < stamp_end; stamp += dt) {
    extended_kalman_filter.predictToTimeAndKeep(stamp);

    std::cout << "covariance matrix:\n" << extended_kalman_filter.covariance() << std::endl;
    plotter_state.printState(extended_kalman_filter.state(), stamp.seconds());
    // plotter_covariance.printVarianceDiagonal(extended_kalman_filter.covariance(), stamp.seconds());
  }

  plotter_covariance.printCovariance(extended_kalman_filter.covariance());
}

TEST(extended_kalman_filter, process_without_system_noise)
{
  // accelerate in x direction without system noise
  // using tested model mecanum to proof update method
  FilterModelMecanum::Parameter filter_parameter;

  filter_parameter.mode_type = FilterModelMecanum::Parameter::ModelType::PUSH_AND_ROTATE;
  filter_parameter.noise.acceleration = 0.0;
  filter_parameter.noise.yaw_rate = 0.0;

  auto filter_model = std::make_unique<FilterModelMecanum>(filter_parameter);
  ExtendedKalmanFilter<FilterModelMecanum::attribute_pack> extended_kalman_filter(std::move(filter_model));
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter_state(
    "extended_kalman_filter_process_without_system_noise_state.csv");
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter_covariance(
    "extended_kalman_filter_process_without_system_noise_cov.csv");

  rclcpp::Time stamp;
  rclcpp::Time stamp_end(10000000000); // 10s
  const rclcpp::Duration dt(0, 10000000); // 10ms

  // measurement used by each iteration
  constexpr Data variance = 0.1 * 0.1;
  AttributeVector<Attribute::ACC_X, Attribute::ACC_Y> measurement;
  Eigen::MatrixX<Data> R = Eigen::MatrixX<Data>::Zero(measurement.size(), measurement.size());
  const auto H = ObservationMatrixHandler::instance().matrix(extended_kalman_filter.state(), measurement);

  std::cout << "observation matrix:\n" << H << std::endl;

  measurement.acceleration_x() = 1.0;
  R(0, 0) = variance;
  R(1, 1) = variance;

  for (rclcpp::Time stamp(0); stamp < stamp_end; stamp += dt) {
    // extended_kalman_filter.process(measurement.get(), R, H, stamp);

    std::cout << "covariance matrix:\n" << extended_kalman_filter.covariance() << std::endl;
    plotter_state.printState(extended_kalman_filter.state(), stamp.seconds());
    plotter_covariance.printVarianceDiagonal(extended_kalman_filter.covariance(), stamp.seconds());
  }

  // plotter_covariance.printCovariance(extended_kalman_filter.covariance());
}

TEST(extended_kalman_filter, process_with_system_noise)
{
  // accelerate in x direction with system noise
  constexpr Data acceleration_std_dev = 0.3;
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution d{0.0, acceleration_std_dev};

  // using tested model mecanum to proof update method
  FilterModelMecanum::Parameter filter_parameter;

  filter_parameter.mode_type = FilterModelMecanum::Parameter::ModelType::PUSH_AND_ROTATE;
  filter_parameter.noise.acceleration = acceleration_std_dev * acceleration_std_dev;
  filter_parameter.noise.yaw_rate = 0.0;

  auto filter_model = std::make_unique<FilterModelMecanum>(filter_parameter);
  ExtendedKalmanFilter<FilterModelMecanum::attribute_pack> extended_kalman_filter(std::move(filter_model));
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter_state(
    "extended_kalman_filter_process_with_system_noise_state.csv");
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter_covariance(
    "extended_kalman_filter_process_with_system_noise_cov.csv");

  rclcpp::Time stamp;
  rclcpp::Time stamp_end(10000000000); // 10s
  const rclcpp::Duration dt(0, 10000000); // 10ms

  // measurement used by each iteration
  constexpr Data variance = acceleration_std_dev * acceleration_std_dev;
  AttributeVector<Attribute::ACC_X, Attribute::ACC_Y> measurement;
  Eigen::MatrixX<Data> R = Eigen::MatrixX<Data>::Zero(measurement.size(), measurement.size());
  const auto H = ObservationMatrixHandler::instance().matrix(extended_kalman_filter.state(), measurement);

  std::cout << "observation matrix:\n" << H << std::endl;

  R(0, 0) = variance;
  R(1, 1) = variance;

  for (rclcpp::Time stamp(0); stamp < stamp_end; stamp += dt) {
    measurement.acceleration_x() = d(gen) + 1.0;
    // extended_kalman_filter.process(measurement.get(), R, H, stamp);

    std::cout << "covariance matrix:\n" << extended_kalman_filter.covariance() << std::endl;
    plotter_state.printState(extended_kalman_filter.state(), stamp.seconds());
    plotter_covariance.printVarianceDiagonal(extended_kalman_filter.covariance(), stamp.seconds());
  }

  // plotter_covariance.printCovariance(extended_kalman_filter.covariance());
}

TEST(extended_kalman_filter, process_without_system_noise_case2)
{
  // accelerate in x direction with system noise
  // using tested model mecanum to proof update method
  FilterModelMecanum::Parameter filter_parameter;

  filter_parameter.mode_type = FilterModelMecanum::Parameter::ModelType::PUSH_AND_ROTATE;
  filter_parameter.noise.acceleration = 0.0;
  filter_parameter.noise.yaw_rate = 0.0;

  auto filter_model = std::make_unique<FilterModelMecanum>(filter_parameter);
  ExtendedKalmanFilter<FilterModelMecanum::attribute_pack> extended_kalman_filter(std::move(filter_model));
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter_state(
    "extended_kalman_filter_process_without_system_noise_case2_state.csv");
  AttributeVectorPlotter<FilterModelMecanum::attribute_vector> plotter_covariance(
    "extended_kalman_filter_process_without_system_noise_case2_cov.csv");

  rclcpp::Time stamp;
  rclcpp::Time stamp_end(10000000000); // 10s
  const rclcpp::Duration dt(0, 10000000); // 10ms

  // measurement used by each iteration
  constexpr Data variance = 0.1 * 0.1;
  AttributeVector<Attribute::W_POS_X, Attribute::W_POS_Y> measurement;
  Eigen::MatrixX<Data> R = Eigen::MatrixX<Data>::Zero(measurement.size(), measurement.size());
  const auto H = ObservationMatrixHandler::instance().matrix(extended_kalman_filter.state(), measurement);

  std::cout << "observation matrix:\n" << H << std::endl;

  R(0, 0) = variance;
  R(1, 1) = variance;

  for (rclcpp::Time stamp(0); stamp < stamp_end; stamp += dt) {
    measurement.x() += 1.0;
    // extended_kalman_filter.process(measurement.get(), R, H, stamp);

    std::cout << "covariance matrix:\n" << extended_kalman_filter.covariance() << std::endl;
    plotter_state.printState(extended_kalman_filter.state(), stamp.seconds());
    plotter_covariance.printVarianceDiagonal(extended_kalman_filter.covariance(), stamp.seconds());
  }

  // plotter_covariance.printCovariance(extended_kalman_filter.covariance());
}