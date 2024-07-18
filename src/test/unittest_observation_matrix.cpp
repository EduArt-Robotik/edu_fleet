#include "edu_fleet/kalman_filter/attribute.hpp"
#include "edu_fleet/kalman_filter/attribute_pack.hpp"
#include <gtest/gtest.h>

#include <edu_fleet/kalman_filter/observation_matrix_handler.hpp>

using eduart::fleet::kalman_filter::Attribute;
using eduart::fleet::kalman_filter::AttributePack;
using eduart::fleet::kalman_filter::ObservationMatrixHandler;

TEST(unittest_observation_matrix, getting_matrix)
{
  const AttributePack<Attribute::POS_X, Attribute::POS_Y, Attribute::VEL_X, Attribute::VEL_Y> pack_full_set;
  const AttributePack<Attribute::VEL_X, Attribute::VEL_Y> pack_velocity;

  const auto matrix = ObservationMatrixHandler::instance().matrix(
    pack_full_set, pack_velocity
  );

  ASSERT_EQ(matrix.rows(), pack_velocity.size());
  ASSERT_EQ(matrix.cols(), pack_full_set.size());

  // row 0
  EXPECT_EQ(matrix(0, 0), 0);
  EXPECT_EQ(matrix(0, 1), 0);
  EXPECT_EQ(matrix(0, 2), 1);
  EXPECT_EQ(matrix(0, 3), 0);

  // row 0
  EXPECT_EQ(matrix(1, 0), 0);
  EXPECT_EQ(matrix(1, 1), 0);
  EXPECT_EQ(matrix(1, 2), 0);
  EXPECT_EQ(matrix(1, 3), 1);
}

TEST(unittest_observation_matrix, getting_matrix_invalid)
{
  const AttributePack<Attribute::POS_X, Attribute::POS_Y, Attribute::VEL_X, Attribute::VEL_Y> pack_full_set;
  const AttributePack<Attribute::ACC_X, Attribute::ACC_Y> pack_acceleration;

  const auto matrix = ObservationMatrixHandler::instance().matrix(
    pack_full_set, pack_acceleration
  );

  ASSERT_EQ(matrix.rows(), pack_acceleration.size());
  ASSERT_EQ(matrix.cols(), pack_full_set.size());

  // row 0
  EXPECT_EQ(matrix(0, 0), 0);
  EXPECT_EQ(matrix(0, 1), 0);
  EXPECT_EQ(matrix(0, 2), 0);
  EXPECT_EQ(matrix(0, 3), 0);

  // row 0
  EXPECT_EQ(matrix(1, 0), 0);
  EXPECT_EQ(matrix(1, 1), 0);
  EXPECT_EQ(matrix(1, 2), 0);
  EXPECT_EQ(matrix(1, 3), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
