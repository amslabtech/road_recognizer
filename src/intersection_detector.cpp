#include "road_recognizer/intersection_detector.h"

Peak::Peak(void)
{
    index = 0;
    width = 0;
    angle = 0;
    angle_diff = 0;
    angle_width = 0;
}

Peak::Peak(int index_)
{
    index = index_;
    width = 0;
    angle = 0;
    angle_diff = 0;
    angle_width = 0;
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
    intersection_flag_pub = nh.advertise<std_msgs::Bool>("/intersection_flag", 1);
    intersection_directions_pub = nh.advertise<std_msgs::Float64MultiArray>("/intersection_directions", 1);

    local_nh.param("EPSILON1", EPSILON1, {0.4});
    local_nh.param("EPSILON2_DIV", EPSILON2_DIV, {8});
    local_nh.param("EPSILON3", EPSILON3, {0.95});
    local_nh.param("MIN_RANGE", MIN_RANGE, {10.0});
    local_nh.param("MIN_WIDTH", MIN_WIDTH, {0.8});
    local_nh.param("MAX_WIDTH", MAX_WIDTH, {8});
    local_nh.param("MAX_ANGLE_WIDTH", MAX_ANGLE_WIDTH, {M_PI});
    local_nh.param("SQUARE_RANGE_RATE", SQUARE_RANGE_RATE, {0.75});
    local_nh.param("OUTPUT_FILE_NAME", OUTPUT_FILE_NAME, {std::string(std::getenv("HOME")) + "data.csv"});

    std::cout << "=== intersection_detector ===" << std::endl;
    std::cout << "EPSILON1: " << EPSILON1 << std::endl;
    std::cout << "EPSILON2_DIV: " << EPSILON2_DIV << std::endl;
    std::cout << "EPSILON3: " << EPSILON3 << std::endl;
    std::cout << "MIN_RANGE: " << MIN_RANGE << std::endl;
    std::cout << "MAX_WIDTH: " << MAX_WIDTH << std::endl;
    std::cout << "MAX_ANGLE_WIDTH: " << MAX_ANGLE_WIDTH << std::endl;
    std::cout << "MIN_WIDTH: " << MIN_WIDTH << std::endl;
    std::cout << "SQUARE_RANGE_RATE: " << SQUARE_RANGE_RATE << std::endl;
    std::cout << "OUTPUT_FILE_NAME: " << OUTPUT_FILE_NAME << std::endl;
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

        std::vector<Peak> peak_list;
        search_peaks(beam_ranges, AVG_RANGE, peak_list);

        // std::cout << "peak candidates: " << std::endl;;
		// cout_peak_candidates(beam_ranges, peak_list);

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

        // std::cout << "peak candidates: " << std::endl;;
		// cout_peak_candidates(beam_ranges, peak_list);

        int peak_num = peak_list.size();

        std::cout << "set peak attribute" << std::endl;
        set_peak_attribute(beam_ranges, peak_list);
        // std::cout << "peak candidates: " << std::endl;;
		// cout_peak_candidates(beam_ranges, peak_list);

        std::vector<int> erase_list;
		
        remove_square_peaks(beam_ranges, erase_list, peak_list, MAX_RANGE);
        clean_peaks(erase_list, peak_list);
        
		// std::cout << "peak candidates: " << std::endl;;
		// cout_peak_candidates(beam_ranges, peak_list);


        // merge peaks
        std::cout << "merge peaks" << std::endl;
        for(int i=0;i<peak_num;i++){
            int j = (i + 1) % peak_num;
            int i1 = peak_list[i].index;
            int i2 = peak_list[j].index;
            int dist = (i2 - i1 + N) % N;
            if(dist < EPSILON2){
                // std::cout << i1 << " and " << i2 << std::endl;
                // std::cout << "merged: " << i1 << " & " << i2 << std::endl;
                if(peak_list[i].width > peak_list[j].width){
                    peak_list[j] = peak_list[i];
                    // std::cout << "merge into " << i1 << " :width" << std::endl;
                    erase_list.push_back(i2);
                }else if(peak_list[i].width < peak_list[j].width){
                    peak_list[i] = peak_list[j];
                    // std::cout << "merge into " << i2 << " :width" << std::endl;
                    erase_list.push_back(i1);
                }
				else{
                    if(beam_ranges[peak_list[i].index] > beam_ranges[peak_list[j].index]){
                        peak_list[j] = peak_list[i];
                        // std::cout << "merge into " << i1 << " :range" << std::endl;
                        erase_list.push_back(i2);
                    }else if(beam_ranges[peak_list[i].index] < beam_ranges[peak_list[j].index]){
                        peak_list[i] = peak_list[j];
                        // std::cout << "merge into " << i2 << " :range" << std::endl;
                        erase_list.push_back(i1);
                    }else{
                        if(peak_list[i].angle_diff > peak_list[j].angle_diff){
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

        // std::cout << "peak candidates: " << std::endl;;
		// cout_peak_candidates(beam_ranges, peak_list);

        // remove peak that has not enough deep valley
        std::cout << "erase not deep peak" << std::endl;
        peak_num = peak_list.size();
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
                erase_list.push_back(i1);
                erase_list.push_back(i2);
            }
        }
        clean_peaks(erase_list, peak_list);

        //remove peak having width smaller than MIN_WIDTH
        std::cout << "erase not wide peak" << std::endl;
        peak_num = peak_list.size();
        for(int i=0;i<peak_num;i++){
            if(peak_list[i].width < MIN_WIDTH){
                // std::cout << "beam " << peak_list[i].index << " will be erased" << std::endl;
                erase_list.push_back(peak_list[i].index);
            }
        }
        clean_peaks(erase_list, peak_list);

        //remove peak having width larger than MAX_WIDTH
        std::cout << "erase too wide peak" << std::endl;
        peak_num = peak_list.size();
        for(int i=0;i<peak_num;i++){
            if(peak_list[i].width > MAX_WIDTH){
                // std::cout << "beam " << peak_list[i].index << " will be erased" << std::endl;
                erase_list.push_back(peak_list[i].index);
            }
        }
        clean_peaks(erase_list, peak_list);

        std::cout << "erase too wide angle peak" << std::endl;
        peak_num = peak_list.size();
        for(int i=0;i<peak_num;i++){
            if(peak_list[i].angle_width > MAX_ANGLE_WIDTH){
                // std::cout << "beam " << peak_list[i].index << " will be erased" << std::endl;
                erase_list.push_back(peak_list[i].index);
            }
        }
        clean_peaks(erase_list, peak_list);

        std::cout << "result peaks: " << std::endl;
		cout_peak_candidates(beam_ranges, peak_list);

        // restore indices
        std::rotate(beam_ranges.rbegin(), beam_ranges.rbegin() + start_index, beam_ranges.rend());
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            it->index = (it->index + start_index) % N;
        }

        visualize_beam(beam_ranges, peak_list);

        ////////////////////////////////////////
        std_msgs::Bool intersection_flag;
        intersection_flag.data = (peak_list.size() > 3);
        intersection_flag_pub.publish(intersection_flag);
        ////////////////////////////////////////
        std_msgs::Float64MultiArray directions;
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            directions.data.push_back(it->index * 2 * M_PI / (double)N - M_PI);
        }
        intersection_directions_pub.publish(directions);
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
        if(beam_ranges[i] > avg && beam_ranges[i] > MIN_RANGE){
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
        it->angle_width = abs(bl - br) * D_THETA;
        it->angle = fabs(atan2(sin(it->angle), cos(it->angle)));
        // std::cout << bl << " -> " << it->index << " -> " << br << std::endl;
        it->width = 0.5 * (dl + dr) * it->angle;
        if(bl >= br){
            it->angle_diff = D_THETA * abs((bl + br) * 0.5 - it->index);
        }else{
            it->angle_diff = D_THETA * abs((bl + br + N) * 0.5 - it->index);
        }
    }
}

void IntersectionDetector::remove_square_peaks(const std::vector<double>& beam_ranges, std::vector<int>& erase_list, std::vector<Peak>& peak_list, const double& MAX_RANGE)
{
	double time = ros::Time::now().toSec();
	const int N = beam_ranges.size();
	const int HARF_N = (double)N * 0.5;
	const int QUARTER_N = (double)N * 0.25;
	std::vector<double> beam_ranges_tmp(
			std::make_move_iterator(beam_ranges.begin()),
			std::make_move_iterator(beam_ranges.begin() + beam_ranges.size()*0.5)
			);
	std::vector<int> peak_list_index;
	for(auto it=peak_list.begin();it!=peak_list.end();it++){
		peak_list_index.push_back(it->index);
	}
	for(int i=0;i<N;++i){
		// std::cout << "==========================" << std::endl;
		int max_range_count = std::count(beam_ranges_tmp.begin(), beam_ranges_tmp.end(), MAX_RANGE);
		// std::cout << "max_range_count: " << max_range_count << std::endl;
		double max_range_rate = (double)max_range_count / (double)HARF_N;
		// std::cout << "max_range_rate : " << max_range_rate << std::endl;
		if(max_range_rate >= SQUARE_RANGE_RATE){
			int center_index = (i + QUARTER_N) % N;
			if(std::find(peak_list_index.begin(), peak_list_index.end(), center_index) != peak_list_index.end()){
                std::cout << "beam " << center_index << " will be erased" << std::endl;
				erase_list.push_back(center_index);
			}
		}
		beam_ranges_tmp.erase(beam_ranges_tmp.begin());
		beam_ranges_tmp.push_back(beam_ranges[(i+HARF_N)%N]);
	}
	std::cout << "removing square peak delay: " << ros::Time::now().toSec() - time << std::endl;
}

void IntersectionDetector::visualize_beam(const std::vector<double>& beam_ranges, const std::vector<Peak>& peak_list)
{
    const int N = beam_ranges.size();
    visualization_msgs::Marker beam_marker;
    beam_marker.header.frame_id = "/velodyne";
    beam_marker.header.stamp = ros::Time::now();
    beam_marker.ns = "intersection_detector";
    beam_marker.id = 0;
    beam_marker.type = visualization_msgs::Marker::LINE_LIST;
    beam_marker.action = visualization_msgs::Marker::ADD;
    beam_marker.lifetime = ros::Duration(0);
    beam_marker.points.reserve(N);
    beam_marker.colors.reserve(N);
    beam_marker.scale.x = 0.05;
    std::string output_data = "angle, range\n";
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
		std::stringstream data;
        if(std::find(peak_list.begin(), peak_list.end(), i) == peak_list.end()){
            c.r = 0;
            c.g = 1;
            c.b = 0;
            c.a = 0.8;
			std::cout << "angle: " << angle << "\trange: " << beam_ranges[i] << std::endl;
			data << angle << "," << beam_ranges[i] << "\n";
        }else{
            c.r = 1;
            c.g = 0;
            c.b = 0;
            c.a = 0.8;
			std::cout << "angle: " << angle << "\trange: " << beam_ranges[i] << " ---------------" << std::endl;
			data << angle << "," << beam_ranges[i] << ",peak" << "\n";
        }
        output_data += data.str();
		
        beam_marker.colors.push_back(c);
        beam_marker.colors.push_back(c);
    }
	std::ofstream ofs(OUTPUT_FILE_NAME);
    if(ofs){
        ofs << output_data;
        ofs.close();
        std::cout << "data saved as " << OUTPUT_FILE_NAME << std::endl;
    }else{
        std::cout << "cannot open file" << std::endl;
    }
    beam_pub.publish(beam_marker);
}

void IntersectionDetector::clean_peaks(std::vector<int>& erase_list, std::vector<Peak>& peak_list)
{
    std::cout << "cleaning" << std::endl;
    for(auto it=peak_list.begin();it!=peak_list.end();){
        // if index is found
        if(std::find(erase_list.begin(), erase_list.end(), it->index) != erase_list.end()){
            it = peak_list.erase(it);
        }else{
            ++it;
        }
    }
    erase_list.clear();
    peak_list.erase(std::unique(peak_list.begin(), peak_list.end()), peak_list.end());
}

void IntersectionDetector::cout_peak_candidates(const std::vector<double>& beam_ranges, std::vector<Peak>& peak_list)
{
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            std::cout << it->index << ", " << it->width << ", " << it->angle << ", " << it->angle_diff << ", " << beam_ranges[it->index] << std::endl;;
        }

}

void IntersectionDetector::process(void)
{
    ros::spin();
}
