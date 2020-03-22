#include <gtest/gtest.h>
#include <Eigen/Dense>

using Eigen::MatrixXf;
using Eigen::Matrix2f;
using Eigen::Matrix3f;

TEST(FASTSLAM_TEST, basic_matrix_assign) {
  MatrixXf m(2,2);
  m = MatrixXf::Ones(2,2);
  EXPECT_NO_THROW(Matrix2f m2 = m);
  EXPECT_DEATH(Matrix3f m3 = m, "Invalid sizes when resizing a matrix or array.");
}
