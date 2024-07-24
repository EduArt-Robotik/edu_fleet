#include <gtest/gtest.h>

#include <edu_fleet/kalman_filter/attribute.hpp>
#include <edu_fleet/kalman_filter/attribute_vector.hpp>

using eduart::fleet::kalman_filter::Attribute;
using eduart::fleet::kalman_filter::AttributeVector;

TEST(attribute_vector, element_access)
{
  AttributeVector<Attribute::W_POS_X, Attribute::W_POS_Y, Attribute::VEL_X, Attribute::VEL_Y> vector;

  vector.x() = 1.0;
  vector.y() = 2.0;
  vector.velocity_x() = 3.0;
  vector.velocity_y() = 4.0;

  // check if data were assigned correctly
  EXPECT_EQ(vector.x(), 1.0);
  EXPECT_EQ(vector.y(), 2.0);
  EXPECT_EQ(vector.velocity_x(), 3.0);
  EXPECT_EQ(vector.velocity_y(), 4.0);

  // check if data order is correct
  EXPECT_EQ(vector.data()[0], 1.0);
  EXPECT_EQ(vector.data()[1], 2.0);
  EXPECT_EQ(vector.data()[2], 3.0);
  EXPECT_EQ(vector.data()[3], 4.0);
}

// int main(int argc, char** argv)
// {
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
