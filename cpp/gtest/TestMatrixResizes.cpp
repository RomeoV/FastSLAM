#include <gtest/gtest.h>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;

TEST(FASTSLAM_TEST, basic_matrix_assign) {
  MatrixXd m(2,2);
  m = MatrixXd::Ones(2,2);
  EXPECT_NO_THROW(Matrix2d m2 = m);
  EXPECT_DEATH(Matrix3d m3 = m, "Invalid sizes when resizing a matrix or array.");
}
