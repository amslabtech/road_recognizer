/**
 * @file peak_detector.h
 * @author amsl
 */
#ifndef __ROAD_RECOGNIZER_PEAK_DETECTOR_H
#define __ROAD_RECOGNIZER_PEAK_DETECTOR_H

// reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4378557
// reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6005622

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace road_recognizer
{
class Peak
{
public:
    Peak(void);
    Peak(int);

    bool operator==(const Peak&) const;
    bool operator!=(const Peak&) const;

    int index_;
    // Arc length of the peak [m]
    double width_;
    double angle_;
    // Absolute angle differcne from the center of peak [rad]
    double angle_diff_;

private:
};

class PeakDetector
{
public:
    PeakDetector(void);

    void set_parameters(double epsilon1, double epsilon2_div, double epsilon3, double min_range, double min_width);
    std::vector<Peak> detect_peaks(const std::vector<double>& beam_ranges);

private:
    std::vector<Peak> search_peaks(const std::vector<double>& beam_ranges, double avg);
    void set_peak_attribute(const std::vector<double>& beam_ranges, std::vector<Peak>& peak_list);
    void clean_peaks(std::vector<int>& erase_list, std::vector<Peak>& peak_list);

    double EPSILON1_;
    double EPSILON2_DIV_;
    double EPSILON3_;
    double MIN_RANGE_;
    double MIN_WIDTH_;
    bool PEAK_ONLY_;

    std::vector<double> beam_ranges_;
};
}
#endif// __ROAD_RECOGNIZER_PEAK_DETECTOR_H 
