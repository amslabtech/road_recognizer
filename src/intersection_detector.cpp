#include "road_recognizer/intersection_detector.h"

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
        int start_index = 0;
        for(;start_index<N;start_index++){
            if(beam_ranges[start_index] < AVG_RANGE){
                break;
            }
        }
        std::rotate(beam_ranges.begin(), beam_ranges.begin() + start_index, beam_ranges.end());
        std::cout << "start index: " << start_index << std::endl;

        std::vector<int> peak_indices;
        search_peaks(beam_ranges, AVG_RANGE, peak_indices);

        std::cout << "peak candidates: ";
        for(auto it=peak_indices.begin();it!=peak_indices.end();++it){
            std::cout << *it << ", ";
        }
        std::cout << std::endl;

        // remove short peak
        for(auto it=peak_indices.begin();it!=peak_indices.end();){
            std::cout << *it << std::endl;
            if(beam_ranges[*it] < EPSILON1 * MAX_RANGE){
                it = peak_indices.erase(it);
            }else{
                ++it;
            }
        }
        int peak_indices_num = peak_indices.size();
        // merge close peaks
        for(int i=0;i<peak_indices_num;i++){
            int i1 = peak_indices[i];
            int i2 = peak_indices[(i+1)%peak_indices_num];
            if(abs(i1 - i2) < EPSILON2){
                // std::cout << "merged: " << i1 << " & " << i2 << std::endl;
                double larger_range = std::max(beam_ranges[i1], beam_ranges[i2]);
                beam_ranges[i1] = larger_range;
                beam_ranges[i2] = larger_range;
                peak_indices[(i+1)%peak_indices_num] = i1;
            }
        }
        peak_indices.erase(std::unique(peak_indices.begin(), peak_indices.end()), peak_indices.end());

        // remove peak that has not enough deep valley
        std::vector<int> erase_indices;
        for(int i=0;i<peak_indices_num;i++){
            int i1 = peak_indices[i];
            int i2 = peak_indices[(i+1)%peak_indices_num];
            if(i1 > i2){
                i2 += peak_indices_num;
            }
            double d_sum = 0;
            for(int j=i1;j<i2;j++){
                d_sum += beam_ranges[j%N];
            }
            double d_avg = d_sum / (double)(i2 - i1 + 1);
            double depth = 2 * d_avg / (double)(beam_ranges[i1] + beam_ranges[i2%N]);
            if(depth > EPSILON3){
                erase_indices.push_back(i);
            }
        }
        int erase_indices_num = erase_indices.size();
        for(int i=0;i<erase_indices_num;i++){
            // set N to remove
            peak_indices[erase_indices[i]] = N;
        }
        peak_indices.erase(std::remove(peak_indices.begin(), peak_indices.end(), N), peak_indices.end());
        peak_indices_num = peak_indices.size();
        std::cout << "result peak indices num: " << peak_indices_num << std::endl;
        visualize_beam(beam_ranges, peak_indices);
    }else{
        std::cout << "\033[31mbeam_ranges is empty\033[0m" << std::endl;
    }
    std::cout << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
}

void IntersectionDetector::search_peaks(const std::vector<double>& beam_ranges, double avg, std::vector<int>& peak_indices)
{
    peak_indices.clear();
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
                peak_indices.push_back(i);
            }
        }
    }
}

void IntersectionDetector::visualize_beam(const std::vector<double>& beam_ranges, const std::vector<int>& peak_indices)
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
        if(std::find(peak_indices.begin(), peak_indices.end(), i) == peak_indices.end()){
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

void IntersectionDetector::process(void)
{
    ros::spin();
}
