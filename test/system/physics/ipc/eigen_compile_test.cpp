#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <gtest/gtest.h>

namespace {

TEST(EigenCompileTest, DenseAndSparseHeadersAreAvailable) {
    static_assert(sizeof(Eigen::VectorXd) > 0);

    Eigen::Vector3d v = Eigen::Vector3d::UnitX();
    Eigen::SparseMatrix<double> sparse(3, 3);

    EXPECT_EQ(v.x(), 1.0);
    EXPECT_EQ(sparse.rows(), 3);
    EXPECT_EQ(sparse.cols(), 3);
}

}  // namespace
