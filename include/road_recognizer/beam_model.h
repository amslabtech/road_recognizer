/**
 * @file beam_model.h
 * @author amsl
 */
#ifndef __ROAD_RECOGNIZER_BEAM_MODEL_H
#define __ROAD_RECOGNIZER_BEAM_MODEL_H

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

class BeamModel
{
public:
    BeamModel(void);

    std::vector<Peak> detect_peaks(const std::vector<double>&);

private:
    std::vector<Peak> search_peaks(const std::vector<double>&, double);
    void set_peak_attribute(const std::vector<double>&, std::vector<Peak>&);
    void clean_peaks(std::vector<int>&, std::vector<Peak>&);

    double EPSILON1_;
    double EPSILON2_DIV_;
    double EPSILON3_;
    double MIN_RANGE_;
    double MIN_WIDTH_;
    bool PEAK_ONLY_;

    std::vector<double> beam_ranges_;
};
}
#endif// __ROAD_RECOGNIZER_BEAM_MODEL_H 
