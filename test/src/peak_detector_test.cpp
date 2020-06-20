#include <gtest/gtest.h>
#include <chrono>
#include "road_recognizer/peak_detector.h"

TEST(PeakDetectionTest, DetectPeak)
{
    road_recognizer::PeakDetector beam_model;
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
    auto start = std::chrono::system_clock::now();
    std::vector<road_recognizer::Peak> peaks = beam_model.detect_peaks(beam_ranges);
    auto end = std::chrono::system_clock::now();
    ASSERT_EQ(peaks.size(), 2);
    double time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << time << "[us]" << std::endl;
    for(auto it=peaks.begin();it!=peaks.end();++it){
        std::cout << it->index_ << ", " << it->width_ << ", " << it->angle_ << ", " << it->angle_diff_ << ", " << beam_ranges[it->index_] << std::endl;;
    }
}

TEST(PeakDetectionTest, DetectPeakInHalfCircle)
{
    road_recognizer::PeakDetector beam_model;
    const unsigned int SIZE = 120;
    const double DTHETA = 2 * M_PI / (double)SIZE;
    std::vector<double>beam_ranges(SIZE);
    unsigned int i = 0;
    for(;i<SIZE/2;i++){
        beam_ranges[i] = 5.0 / sin(i * DTHETA);
    }
    for(;i<SIZE;i++){
        beam_ranges[i] = 20.0;
    }
    auto start = std::chrono::system_clock::now();
    std::vector<road_recognizer::Peak> peaks = beam_model.detect_peaks(beam_ranges);
    auto end = std::chrono::system_clock::now();
    ASSERT_EQ(peaks.size(), 0);
    double time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << time << "[us]" << std::endl;
    for(auto it=peaks.begin();it!=peaks.end();++it){
        std::cout << it->index_ << ", " << it->width_ << ", " << it->angle_ << ", " << it->angle_diff_ << ", " << beam_ranges[it->index_] << std::endl;;
    }
}

TEST(PeakDetectionTest, DetectPeakInCircle)
{
    road_recognizer::PeakDetector beam_model;
    const unsigned int SIZE = 120;
    std::vector<double>beam_ranges(SIZE);
    unsigned int i = 0;
    for(;i<SIZE;i++){
        beam_ranges[i] = 20.0;
    }
    auto start = std::chrono::system_clock::now();
    std::vector<road_recognizer::Peak> peaks = beam_model.detect_peaks(beam_ranges);
    auto end = std::chrono::system_clock::now();
    ASSERT_EQ(peaks.size(), 0);
    double time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << time << "[us]" << std::endl;
    for(auto it=peaks.begin();it!=peaks.end();++it){
        std::cout << it->index_ << ", " << it->width_ << ", " << it->angle_ << ", " << it->angle_diff_ << ", " << beam_ranges[it->index_] << std::endl;;
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}