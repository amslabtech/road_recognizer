#include "road_recognizer/intersection_detector.h"

Peak::Peak(void)
{
    index = 0;
    width = 0;
    angle = 0;
    angle_diff = 0;
    erase_flag = false;
}

Peak::Peak(int index_)
{
    index = index_;
    width = 0;
    angle = 0;
    angle_diff = 0;
    erase_flag = false;
}

bool Peak::operator==(const Peak& p) const
{
    return index == p.index;
}

bool Peak::operator!=(const Peak& p) const
{
    return index != p.index;
}

IntersectionDetector::IntersectionDetector(void)
:local_nh("~")
{
    beam_sub = nh.subscribe("beam_array", 1, &IntersectionDetector::beam_callback, this);
    beam_pub = local_nh.advertise<visualization_msgs::Marker>("beam", 1);

    local_nh.param("EPSILON1", EPSILON1, {0.25});
    local_nh.param("EPSILON2_DIV", EPSILON2_DIV, {8});
    local_nh.param("EPSILON3", EPSILON3, {0.8});
    std::cout << "=== intersection_detector ===" << std::endl;
    std::cout << "EPSILON1: " << EPSILON1 << std::endl;
    std::cout << "EPSILON2_DIV: " << EPSILON2_DIV << std::endl;
    std::cout << "EPSILON3: " << EPSILON3 << std::endl;
}

void IntersectionDetector::beam_callback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    std::cout << "--- beam_callback ---" << std::endl;
    double start_time = ros::Time::now().toSec();
    std::vector<double> beam_ranges = msg->data;
    const int N = beam_ranges.size();
    std::cout << "number of beam: " << N << std::endl;
    // angle: -pi -> pi
    // index:   0 -> N
    const double EPSILON2 = N / EPSILON2_DIV;
    if(N > 0){
        const double AVG_RANGE = std::accumulate(beam_ranges.begin(), beam_ranges.end(), 0.0) / (double)beam_ranges.size();
        std::cout << "average beam range: " << AVG_RANGE << std::endl;
        const double MAX_RANGE = *(std::max_element(beam_ranges.begin(), beam_ranges.end()));
        std::cout << "max beam range: " << MAX_RANGE << std::endl;

        // int start_index = 0;
        // for(;start_index<N;start_index++){
        //     if(beam_ranges[start_index] < AVG_RANGE){
        //         break;
        //     }
        // }
        // std::rotate(beam_ranges.begin(), beam_ranges.begin() + start_index, beam_ranges.end());
        // std::cout << "start index: " << start_index << std::endl;

        std::vector<Peak> peak_list;
        search_peaks(beam_ranges, AVG_RANGE, peak_list);

        std::cout << "peak candidates: " << std::endl;;
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            std::cout << it->index << ", " << it->width << ", " << it->angle << ", " << it->angle_diff << ", " << beam_ranges[it->index] << std::endl;;
        }

        // remove short peak
        std::cout << "remove beam shorter than " << MAX_RANGE * EPSILON1 << std::endl;
        for(auto it=peak_list.begin();it!=peak_list.end();){
            // std::cout << it->index << std::endl;
            if(beam_ranges[it->index] < EPSILON1 * MAX_RANGE){
                it = peak_list.erase(it);
            }else{
                ++it;
            }
        }

        std::cout << "peak candidates: " << std::endl;;
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            std::cout << it->index << ", " << it->width << ", " << it->angle << ", " << it->angle_diff << ", " << beam_ranges[it->index] << std::endl;;
        }

        int peak_num = peak_list.size();

        std::cout << "set peak attribute" << std::endl;
        set_peak_attribute(beam_ranges, peak_list);
        std::cout << "peak candidates: " << std::endl;;
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            std::cout << it->index << ", " << it->width << ", " << it->angle << ", " << it->angle_diff << ", " << beam_ranges[it->index] << std::endl;;
        }
        // merge peaks
        std::cout << "merge peaks" << std::endl;
        std::vector<int> erase_list;
        for(int i=0;i<peak_num;i++){
            int j = (i + 1) % peak_num;
            int i1 = peak_list[i].index;
            int i2 = peak_list[j].index;
            if(abs(i1 - i2) < EPSILON2){
                std::cout << i1 << " and " << i2 << std::endl;
                // std::cout << "merged: " << i1 << " & " << i2 << std::endl;
                if(peak_list[i].width > peak_list[j].width){
                    peak_list[j] = peak_list[i];
                    peak_list[j].erase_flag = true;
                    std::cout << "merge into " << i1 << " :width" << std::endl;
                    erase_list.push_back(i2);
                }else if(peak_list[i].width < peak_list[j].width){
                    peak_list[i] = peak_list[j];
                    peak_list[i].erase_flag = true;
                    std::cout << "merge into " << i2 << " :width" << std::endl;
                    erase_list.push_back(i1);
                }else{
                    if(beam_ranges[peak_list[i].index] > beam_ranges[peak_list[j].index]){
                        peak_list[j] = peak_list[i];
                        peak_list[j].erase_flag = true;
                        std::cout << "merge into " << i1 << " :range" << std::endl;
                        erase_list.push_back(i2);
                    }else if(beam_ranges[peak_list[i].index] < beam_ranges[peak_list[j].index]){
                        peak_list[i] = peak_list[j];
                        peak_list[i].erase_flag = true;
                        std::cout << "merge into " << i2 << " :range" << std::endl;
                        erase_list.push_back(i1);
                    }else{
                        if(peak_list[i].angle_diff > peak_list[j].angle_diff){
                            peak_list[i] = peak_list[j];
                            peak_list[i].erase_flag = true;
                            std::cout << "merge into " << i2 << " :angle_diff" << std::endl;
                            erase_list.push_back(i1);
                        }else{
                            peak_list[j] = peak_list[i];
                            peak_list[j].erase_flag = true;
                            std::cout << "merge into " << i1 << " :angle_diff" << std::endl;
                            erase_list.push_back(i2);
                        }
                    }
                }
            }
        }
        clean_peaks(peak_list);
        // for(auto it=peak_list.begin();it!=peak_list.end();){
        //     if(std::find(erase_list.begin(), erase_list.end(), it->index) == erase_list.end()){
        //         it = peak_list.erase(it);
        //     }else{
        //         ++it;
        //     }
        // }
        std::cout << "peak candidates: " << std::endl;;
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            std::cout << it->index << ", " << it->width << ", " << it->angle << ", " << it->angle_diff << ", " << beam_ranges[it->index] << std::endl;;
        }

        // remove peak that has not enough deep valley
        std::cout << "erase not deep peak" << std::endl;
        for(int i=0;i<peak_num;i++){
            int i1 = peak_list[i].index;
            int i2 = peak_list[(i+1)%peak_num].index;
            if(i1 > i2){
                i2 += peak_num;
            }
            double d_sum = 0;
            for(int j=i1;j<i2;j++){
                d_sum += beam_ranges[j%N];
            }
            double d_avg = d_sum / (double)(i2 - i1 + 1);
            double depth = 2 * d_avg / (double)(beam_ranges[i1] + beam_ranges[i2%N]);
            if(depth > EPSILON3){
                peak_list[i].erase_flag = true;
            }
        }
        clean_peaks(peak_list);
        std::cout << "result peaks: " << std::endl;
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            std::cout << it->index << ", " << it->width << ", " << it->angle << ", " << it->angle_diff << ", " << beam_ranges[it->index] << std::endl;;
        }
        visualize_beam(beam_ranges, peak_list);
    }else{
        std::cout << "\033[31mbeam_ranges is empty\033[0m" << std::endl;
    }
    std::cout << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
}

void IntersectionDetector::search_peaks(const std::vector<double>& beam_ranges, double avg, std::vector<Peak>& peak_list)
{
    peak_list.clear();
    const int N = beam_ranges.size();
    for(int i=0;i<N;i++){
        if(beam_ranges[i] > avg){
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
}

void IntersectionDetector::set_peak_attribute(const std::vector<double>& beam_ranges, std::vector<Peak>& peak_list)
{
    const int N = beam_ranges.size();
    const double D_THETA = 2.0 * M_PI / (double)N;
    for(auto it=peak_list.begin();it!=peak_list.end();++it){
        const int SEARCH_LIMIT = N * 0.5;
        const double HALF_RANGE = beam_ranges[it->index] * 0.5;
        // search CCW(left)
        const int CCW_LIMIT = it->index + SEARCH_LIMIT;
        int i=it->index;
        for(;i<CCW_LIMIT;i++){
            if(beam_ranges[(i+N)%N] < HALF_RANGE){
                break;
            }
        }
        // bl, br = [0, N-1]
        int bl = i%N;
        double dl = beam_ranges[bl];
        // search CW(right)
        const int CW_LIMIT = it->index - SEARCH_LIMIT;
        i=it->index;
        for(;i>CW_LIMIT;i--){
            if(beam_ranges[(i+N)%N] < HALF_RANGE){
                break;
            }
        }
        int br = i%N;
        double dr = beam_ranges[br];
        it->angle = abs(bl - br) * D_THETA;
        it->angle = fabs(atan2(sin(it->angle), cos(it->angle)));
        std::cout << bl << " -> " << it->index << " -> " << br << std::endl;
        it->width = 0.5 * (dl + dr) * it->angle;
        if(bl >= br){
            it->angle_diff = D_THETA * abs((bl + br) * 0.5 - it->index);
        }else{
            it->angle_diff = D_THETA * abs((bl + br + N) * 0.5 - it->index);
        }
    }
}

void IntersectionDetector::visualize_beam(const std::vector<double>& beam_ranges, const std::vector<Peak>& peak_list)
{
    const int N = beam_ranges.size();
    visualization_msgs::Marker beam_marker;
    beam_marker.header.frame_id = "base_link";
    beam_marker.header.stamp = ros::Time::now();
    beam_marker.ns = "intersection_detector";
    beam_marker.id = 0;
    beam_marker.type = visualization_msgs::Marker::LINE_LIST;
    beam_marker.action = visualization_msgs::Marker::ADD;
    beam_marker.lifetime = ros::Duration(0);
    beam_marker.points.reserve(N);
    beam_marker.colors.reserve(N);
    beam_marker.scale.x = 0.1;
    for(int i=0;i<N;i++){
        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;
        beam_marker.points.push_back(p);
        double angle = i / (double)N * 2.0 * M_PI  - M_PI;
        p.x = beam_ranges[i] * cos(angle);
        p.y = beam_ranges[i] * sin(angle);
        beam_marker.points.push_back(p);

        std_msgs::ColorRGBA c;
        if(std::find(peak_list.begin(), peak_list.end(), i) == peak_list.end()){
            c.r = 0;
            c.g = 1;
            c.b = 0;
            c.a = 0.8;
        }else{
            c.r = 1;
            c.g = 0;
            c.b = 0;
            c.a = 0.8;
        }
        beam_marker.colors.push_back(c);
        beam_marker.colors.push_back(c);
    }
    beam_pub.publish(beam_marker);
}

void IntersectionDetector::clean_peaks(std::vector<Peak>& peak_list)
{
    std::cout << "cleaning" << std::endl;
    for(auto it=peak_list.begin();it!=peak_list.end();){
        std::cout << it->index << ", " << it->width << ", " << it->angle << ", " << it->angle_diff << ", " << it->erase_flag << std::endl;;
        if(it->erase_flag == true){
            it = peak_list.erase(it);
        }else{
            ++it;
        }
    }
}

void IntersectionDetector::process(void)
{
    ros::spin();
}
