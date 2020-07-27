#include "road_recognizer/peak_detector.h"

namespace road_recognizer
{
Peak::Peak(void)
{
    index_ = 0;
    width_ = 0;
    angle_ = 0;
    angle_diff_ = 0;
}

Peak::Peak(int index)
{
    index_= index;
    width_ = 0;
    angle_ = 0;
    angle_diff_ = 0;
}

bool Peak::operator==(const Peak& p) const
{
    return index_ == p.index_;
}

bool Peak::operator!=(const Peak& p) const
{
    return index_ != p.index_;
}

PeakDetector::PeakDetector(void)
: EPSILON1_(0.25)
, EPSILON2_DIV_(8)
, EPSILON3_(0.95)
, MIN_RANGE_(10.0)
, MIN_WIDTH_(0.8)
, PEAK_ONLY_(false)
{
}

void PeakDetector::set_parameters(double epsilon1, double epsilon2_div, double epsilon3, double min_range, double min_width)
{
    EPSILON1_ = epsilon1;
    EPSILON2_DIV_ = epsilon2_div;
    EPSILON3_ = epsilon3;
    MIN_RANGE_ = min_range;
    MIN_WIDTH_ = min_width;
}

std::vector<Peak> PeakDetector::detect_peaks(const std::vector<double>& beam_ranges)
{
    beam_ranges_ = beam_ranges;
    const int N = beam_ranges_.size();
    // std::cout << "number of beam: " << N << std::endl;
    // angle: -pi -> pi
    // index:   0 -> N
    const double EPSILON2 = N / EPSILON2_DIV_;
    if(N > 0){
        const double AVG_RANGE = std::accumulate(beam_ranges_.begin(), beam_ranges_.end(), 0.0) / (double)beam_ranges_.size();
        // std::cout << "average beam range: " << AVG_RANGE << std::endl;
        const double MAX_RANGE = *(std::max_element(beam_ranges_.begin(), beam_ranges_.end()));
        // std::cout << "max beam range: " << MAX_RANGE << std::endl;

        int start_index = 0;
        for(;start_index<N;start_index++){
            if(beam_ranges_[start_index] < AVG_RANGE){
                break;
            }
        }
        std::rotate(beam_ranges_.begin(), beam_ranges_.begin() + start_index, beam_ranges_.end());
        // std::cout << "start index: " << start_index << std::endl;

        std::vector<Peak> peak_list = search_peaks(beam_ranges_, AVG_RANGE);

        // std::cout << "peak candidates: " << std::endl;;
        // for(auto it=peak_list.begin();it!=peak_list.end();++it){
        //     std::cout << it->index_ << ", " << it->width_ << ", " << it->angle_ << ", " << it->angle_diff_ << ", " << beam_ranges_[it->index_] << std::endl;;
        // }

        // remove short peak
        // std::cout << "remove beam shorter than " << MAX_RANGE * EPSILON1_ << std::endl;
        for(auto it=peak_list.begin();it!=peak_list.end();){
            // std::cout << it->index_ << std::endl;
            if(beam_ranges_[it->index_] < EPSILON1_ * MAX_RANGE){
                it = peak_list.erase(it);
            }else{
                ++it;
            }
        }

        // std::cout << "peak candidates: " << std::endl;;
        // for(auto it=peak_list.begin();it!=peak_list.end();++it){
        //     std::cout << it->index_ << ", " << it->width_ << ", " << it->angle_ << ", " << it->angle_diff_ << ", " << beam_ranges_[it->index_] << std::endl;;
        // }

        int peak_num = peak_list.size();

        // std::cout << "set peak attribute" << std::endl;
        set_peak_attribute(beam_ranges_, peak_list);
        // std::cout << "peak candidates: " << std::endl;;
        // for(auto it=peak_list.begin();it!=peak_list.end();++it){
        //     std::cout << it->index_ << ", " << it->width_ << ", " << it->angle_ << ", " << it->angle_diff_ << ", " << beam_ranges_[it->index_] << std::endl;;
        // }
        // merge peaks
        // std::cout << "merge peaks" << std::endl;
        std::vector<int> erase_list;
        for(int i=0;i<peak_num;i++){
            // next peak candidate
            int j = (i + 1) % peak_num;
            int i1 = peak_list[i].index_;
            int i2 = peak_list[j].index_;
            int dist = i2 - i1;
            if(i1 > i2){
                dist = i2 - i1 + N;
            }
            if(dist < EPSILON2){
                // select wider peak
                // std::cout << i1 << " and " << i2 << std::endl;
                // std::cout << "merged: " << i1 << " & " << i2 << std::endl;
                if(peak_list[i].width_ > peak_list[j].width_){
                    peak_list[j] = peak_list[i];
                    // std::cout << "merge into " << i1 << " :width" << std::endl;
                    erase_list.push_back(i2);
                }else if(peak_list[i].width_ < peak_list[j].width_){
                    peak_list[i] = peak_list[j];
                    // std::cout << "merge into " << i2 << " :width" << std::endl;
                    erase_list.push_back(i1);
                }else{
                    // select lenger peak
                    if(beam_ranges_[peak_list[i].index_] > beam_ranges_[peak_list[j].index_]){
                        peak_list[j] = peak_list[i];
                        // std::cout << "merge into " << i1 << " :range" << std::endl;
                        erase_list.push_back(i2);
                    }else if(beam_ranges_[peak_list[i].index_] < beam_ranges_[peak_list[j].index_]){
                        peak_list[i] = peak_list[j];
                        // std::cout << "merge into " << i2 << " :range" << std::endl;
                        erase_list.push_back(i1);
                    }else{
                        // select angle nearest the center of peak
                        if(peak_list[i].angle_diff_ > peak_list[j].angle_diff_){
                            peak_list[i] = peak_list[j];
                            // std::cout << "merge into " << i2 << " :angle_diff" << std::endl;
                            erase_list.push_back(i1);
                        }else{
                            peak_list[j] = peak_list[i];
                            // std::cout << "merge into " << i1 << " :angle_diff" << std::endl;
                            erase_list.push_back(i2);
                        }
                    }
                }
            }
        }
        clean_peaks(erase_list, peak_list);

        peak_num = peak_list.size();
        // remove invalid peak
        for(int i=0;i<peak_num;i++){
            if(peak_list[i].angle_diff_ > 0.1){
                erase_list.push_back(peak_list[i].index_);
            }
        }
        clean_peaks(erase_list, peak_list);

        // std::cout << "peak candidates: " << std::endl;;
        // for(auto it=peak_list.begin();it!=peak_list.end();++it){
        //     std::cout << it->index_ << ", " << it->width_ << ", " << it->angle_ << ", " << it->angle_diff_ << ", " << beam_ranges_[it->index_] << std::endl;;
        // }

        // remove peak that has not enough deep valley
        // std::cout << "erase not deep peak" << std::endl;
        for(int i=0;i<peak_num;i++){
            int i1 = peak_list[i].index_;
            int i2 = peak_list[(i+1)%peak_num].index_;
            if(i1 > i2){
                i2 += peak_num;
            }
            double d_sum = 0;
            for(int j=i1;j<i2;j++){
                d_sum += beam_ranges_[j%N];
            }
            double d_avg = d_sum / (double)(i2 - i1 + 1);
            double depth = 2 * d_avg / (double)(beam_ranges_[i1] + beam_ranges_[i2%N]);
            if(depth > EPSILON3_){
                erase_list.push_back(i1);
                erase_list.push_back(i2);
            }
        }
        clean_peaks(erase_list, peak_list);

        //remove peak having width smaller than MIN_WIDTH
        // std::cout << "erase not wide peak" << std::endl;
        for(int i=0;i<peak_num;i++){
            if(peak_list[i].width_ < MIN_WIDTH_){
                // std::cout << "beam " << peak_list[i].index_ << " will be erased" << std::endl;
                erase_list.push_back(peak_list[i].index_);
            }
        }
        clean_peaks(erase_list, peak_list);

        // std::cout << "result peaks: " << std::endl;
        // for(auto it=peak_list.begin();it!=peak_list.end();++it){
        //     std::cout << it->index_ << ", " << it->width_ << ", " << it->angle_ << ", " << it->angle_diff_ << ", " << beam_ranges_[it->index_] << std::endl;;
        // }

        // restore indices
        std::rotate(beam_ranges_.rbegin(), beam_ranges_.rbegin() + start_index, beam_ranges_.rend());
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            it->index_ = (it->index_ + start_index) % N;
        }
        auto compare_peak_index = [](const auto& p1, const auto& p2) -> bool
        {
            return (p1.index_ < p2.index_);
        };
        std::sort(peak_list.begin(), peak_list.end(), compare_peak_index);

        return peak_list;
    }else{
        std::cout << "\033[31mbeam_ranges is empty\033[0m" << std::endl;
    }
    return std::vector<Peak>();
}

std::vector<Peak> PeakDetector::search_peaks(const std::vector<double>& beam_ranges, double avg)
{
    std::vector<Peak> peak_list;
    const int N = beam_ranges.size();
    for(int i=0;i<N;i++){
        if(beam_ranges[i] > avg && beam_ranges[i] > MIN_RANGE_){
            // std::cout << "i=" << i << std::endl;
            // std::cout << (i - 1 + N) % N << ": " << beam_ranges[(i - 1 + N) % N] << std::endl;
            // std::cout << i << ": " << beam_ranges[i] << std::endl;
            // std::cout << (i + 1) % N << ": " << beam_ranges[(i + 1) % N] << std::endl;
            // if((beam_ranges[i] > beam_ranges[(i - 1 + N) % N]) && (beam_ranges[i] > beam_ranges[(i + 1) % N])){
            if((beam_ranges[i] >= beam_ranges[(i - 1 + N) % N]) && (beam_ranges[i] >= beam_ranges[(i + 1) % N])){
                // i is a candidate of peak
                peak_list.push_back(Peak(i));
            }
        }
    }
    return peak_list;
}

void PeakDetector::set_peak_attribute(const std::vector<double>& beam_ranges, std::vector<Peak>& peak_list)
{
    const int N = beam_ranges.size();
    const double D_THETA = 2.0 * M_PI / (double)N;
    for(auto it=peak_list.begin();it!=peak_list.end();++it){
        const int SEARCH_LIMIT = N * 0.5;
        const double HALF_RANGE = beam_ranges[it->index_] * 0.5;
        // search CCW(left)
        const int CCW_LIMIT = it->index_ + SEARCH_LIMIT;
        int i=it->index_;
        for(;i<CCW_LIMIT;i++){
            if(beam_ranges[(i+N)%N] < HALF_RANGE){
                break;
            }
        }
        // bl, br = [0, N-1]
        int bl = i%N;
        double dl = beam_ranges[bl];
        // search CW(right)
        const int CW_LIMIT = it->index_ - SEARCH_LIMIT;
        i=it->index_;
        for(;i>CW_LIMIT;i--){
            if(beam_ranges[(i+N)%N] < HALF_RANGE){
                break;
            }
        }
        int br = i%N;
        double dr = beam_ranges[br];
        it->angle_ = abs(bl - br) * D_THETA;
        it->angle_ = fabs(atan2(sin(it->angle_), cos(it->angle_)));
        // std::cout << bl << " -> " << it->index_ << " -> " << br << std::endl;
        it->width_ = 0.5 * (dl + dr) * it->angle_;
        if(bl >= br){
            it->angle_diff_ = D_THETA * abs((bl + br) * 0.5 - it->index_);
        }else{
            it->angle_diff_ = D_THETA * abs((bl + br + N) * 0.5 - it->index_);
        }
    }
}

void PeakDetector::clean_peaks(std::vector<int>& erase_list, std::vector<Peak>& peak_list)
{
    // std::cout << "cleaning" << std::endl;
    for(auto it=peak_list.begin();it!=peak_list.end();){
        // if index_ is found
        if(std::find(erase_list.begin(), erase_list.end(), it->index_) != erase_list.end()){
            it = peak_list.erase(it);
        }else{
            ++it;
        }
    }
    erase_list.clear();
    peak_list.erase(std::unique(peak_list.begin(), peak_list.end()), peak_list.end());
}
}