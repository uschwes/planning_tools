#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>
#include <Eigen/Core>

void exportEigen21DontAlign() {
  NumpyEigenConverter< Eigen::Matrix<double, 2, 1, Eigen::DontAlign> >::register_converter();
  NumpyEigenConverter< Eigen::Matrix<int, 2, 1, Eigen::DontAlign> >::register_converter();
  NumpyEigenConverter< Eigen::Matrix<std::size_t, 2, 1, Eigen::DontAlign> >::register_converter();
}
