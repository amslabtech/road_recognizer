#include <gtest/gtest.h>
#include <ros/ros.h>

#include "road_recognizer/intersection_detector.h"
#include "std_msgs/Float64MultiArray.h"

class IntersectionDetectionTest: public ::testing::Test
{
public:
    IntersectionDetectionTest(void)
    {
        beam_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("beam_array", 1, true);
        peak_sub_ = nh_.subscribe("intersection_directions", 1, &IntersectionDetectionTest::directions_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    }

    void directions_callback(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
        peak_directions_ = msg->data;
        directions_subscribed_ = true;
    }

protected:
    virtual void SetUp()
    {
        peak_directions_.clear();
        directions_subscribed_ = false;
    }

    ros::NodeHandle nh_;
    ros::Publisher beam_pub_;
    ros::Subscriber peak_sub_;
    std::vector<double> peak_directions_;
    bool directions_subscribed_;
};

TEST_F(IntersectionDetectionTest, PubSubTest)
{
    const unsigned int SIZE = 120;
    std::vector<double>beam_ranges(SIZE);
    unsigned int i = 0;
    for(;i<SIZE/4;i++){
        beam_ranges[i] = 5.0;
    }
    for(;i<SIZE/2;i++){
        beam_ranges[i] = 20.0;
    }
    for(;i<2 / 3. * SIZE;i++){
        beam_ranges[i] = 5.0;
    }
    for(;i<3 / 4. * SIZE;i++){
        beam_ranges[i] = 15.0;
    }
    for(;i<SIZE;i++){
        beam_ranges[i] = 5.0;
    }
    std_msgs::Float64MultiArray ranges;
    ranges.data = beam_ranges;
    beam_pub_.publish(ranges);

    while(ros::ok() && !directions_subscribed_){
        std::cout << "loop" << std::endl;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    std::cout << "cb" << std::endl;
    for(auto d : peak_directions_){
        std::cout << d << std::endl;
    }
    ASSERT_EQ(peak_directions_.size(), 2);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "intersection_detection_test");
    return RUN_ALL_TESTS();
}