#include <gtest/gtest.h>
#include <chrono>
#include "road_recognizer/road_shape_estimator.h"

TEST(RoadShapeEstimatorTest, FitSpline)
{
    road_recognizer::RoadShapeEstimator rse;
    std::vector<std::vector<Eigen::Vector2d>> segments;
    segments.resize(1);
    Eigen::Vector2d v;
    for(double x=0.0;x<2.0;x+=0.1){
        v << x, x * x;
        segments[0].push_back(v);
    }
    rse.rasterize(segments);
    std::vector<unsigned int> indices = {0, 1, 2, 4, 8};
    Eigen::MatrixXd points = rse.fit_spline(segments[0], indices);
    std::cout << "points:\n" << points << std::endl;

    const double allowable_error = 0.01;
    ASSERT_NEAR(points(0, 0), segments[0][indices[0]](0), allowable_error);
    ASSERT_NEAR(points(0, 1), segments[0][indices[0]](1), allowable_error);
    ASSERT_NEAR(points(3, 0), segments[0][indices.back()](0), allowable_error);
    ASSERT_NEAR(points(3, 1), segments[0][indices.back()](1), allowable_error);

    const double score = rse.compute_score(points);
    std::cout << "score: " << score << std::endl;
    ASSERT_GT(score, 0.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "road_shape_estimator_test");
    return RUN_ALL_TESTS();
}