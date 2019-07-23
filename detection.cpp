#include "detection.h"

Detection::Detection(){
    pipe.start();
    detectBoard();
    data = getDepth();
}

std::vector<std::vector<double>> Detection::getDepth(){

}

std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int, int, int>>> Detection::singleDetect(){
}

void Detection::detectBoard(){
    auto angle = []( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
    };
    
    auto findSquares = [&]( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares )
    {
        int thresh = 1000, N = 11;
        squares.clear();
        
        cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;
        
        // down-scale and upscale the image to filter out the noise
        cv::pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
        cv::pyrUp(pyr, timg, image.size());
        std::vector<std::vector<cv::Point> > contours;
        
        // find squares in every color plane of the image
        for( int c = 0; c < 3; c++ )
        {
            int ch[] = {c, 0};
            cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);
            
            // try several threshold levels
            for( int l = 0; l < N; l++ )
            {
                // hack: use Canny instead of zero threshold level.
                // Canny helps to catch squares with gradient shading
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
                // find contours and store them all as a list
                findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
                std::vector<cv::Point> approx;
                
                // test each contour
                for( size_t i = 0; i < contours.size(); i++ )
                {
                    // approximate contour with accuracy proportional
                    // to the contour perimeter
                    cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true)*0.02, true);
                    // square contours should have 4 vertices after approximation
                    // relatively large area (to filter out noisy contours)
                    // and be convex
                    // Note: absolute value of an area is used because
                    // area may be positive or negative - in accordance with the
                    // contour orientation
                    if( approx.size() == 4 &&
                    std::fabs(contourArea(approx)) > 1000 &&
                    cv::isContourConvex(approx) )
                    {
                        double maxCosine = 0;

                        for( int j = 2; j < 5; j++ )
                        {
                            // find the maximum cosine of the angle between joint edges
                            double cosine = std::fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                            maxCosine = MAX(maxCosine, cosine);
                        }

                        // if cosines of all angles are small
                        // (all angles are ~90 degree) then write quandrange
                        // vertices to resultant sequence
                        if( maxCosine < 0.3 )
                            squares.push_back(approx);
                    }
                }
            }
        }
    };
    auto frame_to_mat = [](const rs2::frame& f){
        auto vf = f.as<rs2::video_frame>();
        const int w = vf.get_width();
        const int h = vf.get_height();

        if (f.get_profile().format() == RS2_FORMAT_BGR8)
        {
            return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_RGB8)
        {
            auto r = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
            cv::cvtColor(r, r, cv::COLOR_RGB2BGR);
            return r;
        }
        else if (f.get_profile().format() == RS2_FORMAT_Z16)
        {
            return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_Y8)
        {
            return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
        {
            return cv::Mat(cv::Size(w, h), CV_32FC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        }

        throw std::runtime_error("Frame format is not supported yet!");
    };
    BoardPos.clear();
    std::vector<std::vector<cv::Point> > squares;



    rs2::colorizer color_map;
    rs2::rates_printer printer;


    // Wait for the next set of frames from the camera. Now that autoexposure, etc.
    // has settled, we will write these to disk
    

    cv::Mat image = frame_to_mat( pipe.wait_for_frames().    // Wait for next set of frames from the camera
                             apply_filter(printer).     // Print each enabled stream frame rate
                             apply_filter(color_map));

    if( image.empty() ){
      std::cerr << "Couldn't load " << std::endl;
      std::abort();
    }
    
    findSquares(image, squares);

    std::vector<std::pair<int, int>> vec;
    for(auto a : squares){
        for(auto b : a)vec.push_back(std::make_pair(b.x, b.y));
    }
    sort(vec.begin(), vec.end());
    std::vector<std::pair<int, int>> frame = { std::make_pair(0, 0), std::make_pair(0, 0), std::make_pair(0, 0), std::make_pair(0, 0)};
    for(int i = 0;i < vec.size();i++){
      if(i < vec.size()/4){
        frame.at(0).first += vec.at(i).first;
        frame.at(0).second += vec.at(i).second;
      }else if (i < vec.size()/2){
        frame.at(1).first += vec.at(i).first;
        frame.at(1).second += vec.at(i).second;

      }else if (i < vec.size()/4*3){
        frame.at(2).first += vec.at(i).first;
        frame.at(2).second += vec.at(i).second;

      }else{
        frame.at(3).first += vec.at(i).first;
        frame.at(3).second += vec.at(i).second;

      }
    }
    
    frame.at(0).first /= vec.size()/4;
    frame.at(0).second /= vec.size()/4;
    frame.at(1).first /= vec.size()/4;
    frame.at(1).second /= vec.size()/4;
    frame.at(2).first /= vec.size()/4;
    frame.at(2).second /= vec.size()/4;
    frame.at(3).first /= vec.size()/4;
    frame.at(3).second /= vec.size()/4;
    
    std::cout<<"Board pos in piexl"<<std::endl;
    for(int i = 0;i < 4;i++){
      std::cout<<"("<<frame.at(i).first<<" "<<frame.at(i).second<<")";
    }
    std::cout<<std::endl;

    rs2::frameset pframes = pipe.wait_for_frames();
    rs2::depth_frame depth = pframes.get_depth_frame();

    std::vector<std::vector<float>> dpoint(4);
    rs2_intrinsics intr = pframes.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data


    for(int i = 0;i < 4;i++){
        const float qpixel[2] = {frame.at(i).first, frame.at(i).second};
        float qpoint[3];
        rs2_deproject_pixel_to_point(qpoint, &intr, qpixel, depth.get_distance(qpixel[0], qpixel[1]));
        BoardPos.push_back(std::make_tuple(qpoint[0], qpoint[1], qpoint[2]));
        std::cout<<qpoint[0]<<" "<<qpoint[1]<<" "<<qpoint[2]<<std::endl;
    }
}