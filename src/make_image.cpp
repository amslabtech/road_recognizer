#include "road_recognizer/make_image.h"



MakeImage::MakeImage()
{
    w = 40.0;
    h = 40.0;
    resolution = 0.1;
    resolution_rec = 1 / resolution;
    image_w = int(w * resolution_rec);
    image_h = int(h * resolution_rec);
    houghline_flag = true;
	is_precasted = false;
	BEAM_ANGLE_NUM = 360;
	BEAM_NODE_ARM_LENGTH = 5;
	BEAM_NODE_SCALE = 1;
	BEAM_NODE_NUM = BEAM_NODE_ARM_LENGTH / BEAM_NODE_SCALE * 2 + 1;
	BEAM_CENTER_ID = BEAM_NODE_ARM_LENGTH * BEAM_NODE_NUM + BEAM_NODE_ARM_LENGTH;
    ANGLE_INCREMENT = 2.0 * M_PI / (double)BEAM_ANGLE_NUM;
	MAX_BEAM_RANGE = 1023;
}

MakeImage::Precasting::Precasting(int _index, int _cx, int _cy, double _distance)
{
	index = _index;
	cx = _cx;
	cy = _cy;
	distance = _distance;
}

void MakeImage::set_param(const double wid, const double hei, const double res, const int num, const double range, const int beam_arm, const int beam_scale)
{
    w = wid;
    h = hei;
    resolution = res;
    resolution_rec = 1 / resolution;
    image_w = int(w * resolution_rec);
    image_h = int(h * resolution_rec);
	BEAM_ANGLE_NUM = num;
	BEAM_NODE_ARM_LENGTH = beam_arm;
	BEAM_NODE_SCALE = beam_scale;
	BEAM_NODE_NUM = BEAM_NODE_ARM_LENGTH / BEAM_NODE_SCALE * 2 + 1;
	BEAM_CENTER_ID = BEAM_NODE_ARM_LENGTH * BEAM_NODE_NUM + BEAM_NODE_ARM_LENGTH;
    ANGLE_INCREMENT = 2.0 * M_PI / (double)BEAM_ANGLE_NUM;
	MAX_BEAM_RANGE = range;
}

void MakeImage::extract_pc_in_range(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc)/*{{{*/
{
    // std::cout<<"extractpcinrange"<<std::endl;
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(pc);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-w*0.5, w*0.5);
    pass.filter(*pc);
    pass.setInputCloud(pc);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-h*0.5, h*0.5);
    pass.filter(*pc);
}/*}}}*/

void MakeImage::make_image(void)
{
    // std::cout<<"make_image"<<std::endl;
    cv::Mat image(cv::Size(image_w,image_h), CV_8UC1, cv::Scalar(0));
    cv::Mat image_edge(cv::Size(image_w,image_h), CV_8UC1, cv::Scalar(0));
    cv::Mat image_c(cv::Size(image_w,image_h), CV_8UC3, cv::Scalar(0));
    cv::Mat image_c2(cv::Size(image_w,image_h), CV_8UC3, cv::Scalar(0));
    cv::Mat image_c3(cv::Size(image_w,image_h), CV_8UC3, cv::Scalar(0));

    int max = 0;
    add_point_data(grass,image,max);

    ////////////////////////////////////////////////////////////

    // cv::medianBlur(image, image, 7);
    // cv::erode(image, image, cv::Mat(), cv::Point(-1,-1), 1); 
    // cv::dilate(image, image, cv::Mat(), cv::Point(-1,-1), 1); 
    // cv::GaussianBlur(image, image, cv::Size(7,7), 10, 10);// src_img, out_img，karnel_size，標準偏差x, y

    cvtColor(image, image_c3, CV_GRAY2RGB);
    cv::circle(image_c3, cv::Point(image_w * 0.5,image_h * 0.5), 3, cv::Scalar(100,255,0), -1, CV_AA);
    image_raw = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_c3).toImageMsg();
    
	
	cv::Mat element = (cv::Mat_<uchar>(7,7)<< 0,0,1,1,1,0,0,
                                              0,1,1,1,1,1,0,
                                              1,1,1,1,1,1,1,
                                              1,1,1,1,1,1,1,
                                              1,1,1,1,1,1,1,
                                              0,1,1,1,1,1,0,
                                              0,0,1,1,1,0,0);

    cv::dilate(image, image, element, cv::Point(-1,-1), 1); 
    cv::erode(image, image, element, cv::Point(-1,-1), 2); 
    cv::dilate(image, image, element, cv::Point(-1,-1), 3); 
    cv::erode(image, image, element, cv::Point(-1,-1), 4); 
    cv::dilate(image, image, element, cv::Point(-1,-1), 2); 
    
    generate_pcl(image);
	add_point_data_obs(rmground,image);

    
    beam(image, image_edge);


    // normalize(image,max);
    // cv::threshold(image, image, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
    /////////////////////////////////////////////////////////////
    // cv::Canny(image, image, 50, 200, 3); 
    

    cvtColor(image, image_c, CV_GRAY2RGB);
    // if(houghline_flag)hough_line_p(image, image_c);
    cv::circle(image_c, cv::Point(image_w * 0.5,image_h * 0.5), 3, cv::Scalar(100,255,0), -1, CV_AA);
    image_ros = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_c).toImageMsg();
   

    cvtColor(image_edge, image_c2, CV_GRAY2RGB);
    if(houghline_flag)hough_line_p(image_edge, image_c2);
    cv::circle(image_c2, cv::Point(image_w * 0.5, image_h * 0.5), 3, cv::Scalar(100,255,0), -1, CV_AA);
    image_ros2 = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_c2).toImageMsg();
}

void MakeImage::generate_pcl(const cv::Mat& image){/*{{{*/
    grass_points->points.clear();
    pcl::PointXYZI pt;
    pt.z = 0;
    for(int i=0; i<image_h; i++){
        for(int j=0; j<image_w; j++){
            int ind = image_w * j + i; 
            if(image.data[ind]>0){
                pt.x = pixel_y_to_meter_point_x(j);
                pt.y = pixel_x_to_meter_point_y(i);
                grass_points->points.push_back(pt);
            }
        }
    }
    pcl::toROSMsg(*grass_points, grass_pc2);
}/*}}}*/

void MakeImage::precasting(const int id, const int cx, const int cy)
{
    precast[id].resize(BEAM_ANGLE_NUM);

    for(int i=0; i<image_h; i++){
        for(int j=0; j<image_w; j++){
            int ind = i * image_w + j;
            int dx = j - cx;
            int dy = i - cy;
            double distance = sqrt(dx*dx + dy*dy);
            double grid_size_angle = atan2(0.5 * sqrt(2), distance);

            // std::cout << "====================" <<  std::endl;
            // std::cout << "(dx, dy):(" << dx <<", " << dy << ")" <<  std::endl;
            // std::cout << "distance       : " << distance << std::endl;
            // std::cout << "grid_size_angle: " << grid_size_angle << std::endl;
            // std::cout << "ANGLE_INCREMENT: " << ANGLE_INCREMENT << std::endl;
            int grid_beam_width = 0;
            for(int k=1; grid_size_angle>=ANGLE_INCREMENT*k; k++){
                grid_beam_width = k;
				// if(grid_size_angle<=ANGLE_INCREMENT*(k+1))std::cout << "k:" << k << std::endl;
            }
            double angle = atan2(dx, -dy);
            int index = (angle + M_PI) / ANGLE_INCREMENT;
            for(int k=-grid_beam_width; k<=grid_beam_width; k++){
				Precasting pre(ind, cx, cy, distance);
                precast[id][(index+k+BEAM_ANGLE_NUM) % BEAM_ANGLE_NUM].push_back(pre);
            }
        }
    }
	std::cout << "precasting[" << id << "] ended" << std::endl;
}

void MakeImage::precast_manage(const cv::Mat& image)
{
	if(!is_precasted){
		precast.resize(BEAM_NODE_NUM * BEAM_NODE_NUM);
		for(int i=-BEAM_NODE_ARM_LENGTH;i<=BEAM_NODE_ARM_LENGTH;i+=BEAM_NODE_SCALE){
			for(int j=-BEAM_NODE_ARM_LENGTH;j<=BEAM_NODE_ARM_LENGTH;j+=BEAM_NODE_SCALE){
				int id = (i + BEAM_NODE_ARM_LENGTH) * BEAM_NODE_NUM + (j + BEAM_NODE_ARM_LENGTH);
				int cx = image_w * 0.5 + j * resolution_rec * BEAM_NODE_SCALE;
				int cy = image_h * 0.5 + i * resolution_rec * BEAM_NODE_SCALE;
				// std::cout << "(id, cx, cy): (" << id << ",r" << cx << ", " << cy << ")" << std::endl;
				precasting(id, cx, cy);
			}
		}

		is_precasted = true;
	}
	
}

void MakeImage::beam(const cv::Mat& image, cv::Mat& image_edge)
{
	precast_manage(image);
	image_edge = cv::Scalar(0);
    for(int id=0;id<BEAM_NODE_NUM*BEAM_NODE_NUM;id++){
    	std::vector<int> ind_list(BEAM_ANGLE_NUM, 0);
		// std::cout << "=================-" << std::endl;
		// std::cout << precast[id][0][0].cx << std::endl;
		// std::cout << precast[id][0][0].cy << std::endl;
		// std::cout << image.at<uchar>(precast[id][0][0].cy, precast[id][0][0].cx) << std::endl;
		int x = precast[id][0][0].cx;
		int y = precast[id][0][0].cy;
		if(image.at<uchar>(y, x) == 0 &&
			image.at<uchar>((y+image_h*0.5)*0.5, (x+image_w*0.5)*0.5) == 0 &&
			image.at<uchar>((y*3+image_h*0.5)*0.25, (x*3+image_w*0.5)*0.25) == 0 &&
			image.at<uchar>((y+image_h*1.5)*0.25, (x+image_w*1.5)*0.25) == 0
			)
		{
    		for(int beam_angle=0; beam_angle<BEAM_ANGLE_NUM; beam_angle++){
    		    int angle_grid_num = precast[id][beam_angle].size();
    		    double min_dist = MAX_BEAM_RANGE * resolution_rec * resolution_rec;
    		    for(int grid=0; grid<angle_grid_num; grid++){
    		        int ind = precast[id][beam_angle][grid].index;
    		        if(image.data[ind]>0){
    		            double distance = precast[id][beam_angle][grid].distance;
    		            if(min_dist > distance){
							ind_list[beam_angle] = ind;
    		                min_dist = distance;
						}
    		        }
    		    }
    		}
			
			for(int beam_angle=0; beam_angle<BEAM_ANGLE_NUM; beam_angle++){
				image_edge.data[ind_list[beam_angle]] = 255;
			}
		}
	}
    // cv::dilate(image_edge, image_edge, cv::Mat(), cv::Point(-1,-1), 1); 
    // cv::Canny(image_edge, image_edge, 50, 200, 3); 

}

void MakeImage::add_point_data(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, cv::Mat& image, int& max)/*{{{*/
{
    // std::cout<<"add_point_data"<<std::endl;
    if(size_t lim = pc->points.size()){
        for(size_t i=0; i<lim; i++){
            int px_x = meter_point_to_pixel_x(pc->points[i].y);
            int px_y = meter_point_to_pixel_y(pc->points[i].x);
            // double r = sqrt(pc->points[i].x*pc->points[i].x+pc->points[i].y*pc->points[i].y);
            int ind = px_y * image_w + px_x;
            // image.data[ind] += 1;
            // image.data[ind] += int(r*r*0.5+4);
            image.data[ind] = 255;
            if(image.data[ind]>max){
                max = image.data[ind];
                // if(max>200)std::cout<<"max:"<<max<<", x:"<<px_x<< ", y:"<<px_y <<std::endl;
            }
        }
    }
}/*}}}*/

void MakeImage::add_point_data_obs(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, cv::Mat& image)/*{{{*/
{
    // std::cout<<"AddPointData"<<std::endl;
    if(size_t lim = pc->points.size()){
        for(size_t i=0;i<lim;i++){
            int px_x = meter_point_to_pixel_x(pc->points[i].y);
            int px_y = meter_point_to_pixel_y(pc->points[i].x);
            int ind = px_y * image_w + px_x;
            image.data[ind] = 255;
        }
    }
}/*}}}*/

void MakeImage::normalize(cv::Mat& image,const int max)/*{{{*/
{
    // std::cout<<"normalize"<<std::endl;
    size_t lim = image_w * image_h;
    double normalize_n = 255 / max;
    for(size_t i=0; i<lim; i++){
        image.data[i] *= normalize_n;
    }
}/*}}}*/

void MakeImage::amp(cv::Mat& image,const double gain)/*{{{*/
{
    // std::cout<<"amp"<<std::endl;
    size_t lim = image_w * image_h;
    
    for(size_t i=0; i<lim; i++){
        if(image.data[i] * gain < 256)image.data[i] *= gain;
        else image.data[i] = 255;
    }
}/*}}}*/

void MakeImage::hough_line_p(cv::Mat& image,cv::Mat& image_c)
{
    // std::cout<<"hough_line_p"<<std::endl;
    // cv::Canny(image, image, 50, 200, 3); 
    std::vector<cv::Vec4i> lines;
    // 入力画像，出力，距離分解能，角度分解能，閾値，線分の最小長さ，
    // 2点が同一線分上にあると見なす場合に許容される最大距離
    // cv::HoughLinesP(image, lines, 1, CV_PI/180, 40, 40, 20);
    cv::HoughLinesP(image, lines, 1, CV_PI/180, 30, 50, 15);
    std::vector<cv::Vec4i>::iterator it = lines.begin();
    std::cout << "number of lines: " << lines.size() << std::endl;
    for(; it!=lines.end(); ++it) {
        cv::Vec4i l = *it;
        cv::line(image_c, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 2, CV_AA);
    }
}

int MakeImage::meter_point_to_pixel_x(const double y)/*{{{*/
{
    int x_ = -y*resolution_rec + image_w*0.5 - 0.5;
    return x_;
}/*}}}*/

int MakeImage::meter_point_to_pixel_y(const double x)/*{{{*/
{
    int y_ = -x*resolution_rec + image_h*0.5 - 0.5;
    return y_;
}/*}}}*/

double MakeImage::pixel_y_to_meter_point_x(const int y)/*{{{*/
{
    double x_ = -y*resolution + h * 0.5;
    return x_;
}/*}}}*/

double MakeImage::pixel_x_to_meter_point_y(const int x)/*{{{*/
{
    double y_ = -x*resolution + w *0.5;
    return y_;
}/*}}}*/
