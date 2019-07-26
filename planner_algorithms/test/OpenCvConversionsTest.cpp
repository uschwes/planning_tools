#include <gtest/gtest.h>
#include <iostream>
#include <sm/random.hpp>
#include <planner_algorithms/OpenCvConversions.hpp>

using namespace planning2d;

template <typename T>
T rand()
{
  if (std::is_floating_point<T>())
    return sm::random::randLU(std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
  return sm::random::randLUi(std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
}
template <>
OccupancyValue rand<OccupancyValue>()
{
  int v = sm::random::randLUi(0, 2);
  return v == 0 ? OccupancyValue::FREE :(v == 1 ? OccupancyValue::UNKNOWN : OccupancyValue::OCCUPIED);
}

template <typename T>
struct OpenCvConversionTests : public ::testing::Test { virtual ~OpenCvConversionTests() { } };
typedef ::testing::Types<
    bool, OccupancyValue, int8_t, uint8_t, int16_t, uint16_t,
    int32_t, uint32_t, int64_t, uint64_t, float, double>
OpenCvConversionTypes;
TYPED_TEST_CASE(OpenCvConversionTests, OpenCvConversionTypes);

template <typename T>
void testConversion()
{
  planning2d::Map<T> map(Pose2d(0., 0., 0.), 0.1, typename planning2d::Map<T>::Size2d(10, 20), rand<T>());
  cv::Mat mat = toCvMat(map);
  EXPECT_EQ(cv::DataType<T>::type, mat.type());
  ASSERT_EQ(map.matrix().rows(), mat.rows);
  ASSERT_EQ(map.matrix().cols(), mat.cols);

  for (int row = 0; row < map.matrix().rows(); ++row)
  {
    for (int col = 0; col < map.matrix().cols(); ++col)
      ASSERT_EQ(map.matrix()(row, col), mat.at<T>(row, col));
  }
}

TYPED_TEST(OpenCvConversionTests, testConversion)
{
  try
  {
    testConversion<TypeParam>();
  }
  catch (const std::exception& e)
  {
    FAIL() << e.what();
  }
}

TEST(OpenCvConversionTests, testConversion2)
{
  try
  {
    using namespace Eigen;
    Matrix<double, Dynamic, Dynamic, RowMajor> mat = MatrixXd::Zero(3,3);
    {
      cv::Mat cvMat = convertToCvMat<uint8_t>(mat);
      cvMat.at<uint8_t>(0,0) = 1;
      cvMat.at<uint8_t>(0,2) = 2;
      EXPECT_EQ(0, mat(0,0)); // test that copy was done
      EXPECT_EQ(0, mat(0,2)); // test that copy was done
    }
    {
      cv::Mat cvMat = convertToCvMat<double>(mat);
      cvMat.at<double>(0,0) = 1.0;
      cvMat.at<double>(0,2) = 2.0;
      EXPECT_DOUBLE_EQ(1.0, mat(0,0)); // test that no copy was done
      EXPECT_DOUBLE_EQ(2.0, mat(0,2)); // test that no copy was done
    }
  }
  catch (const std::exception& e)
  {
    FAIL() << e.what();
  }
}
