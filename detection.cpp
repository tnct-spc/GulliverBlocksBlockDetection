#include "detection.h"

Detection::Detection(){
    pipe.start();
    detectBoard();
}

std::vector<std::vector<double>> Detection::getDepth(){

}

std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int, int, int>>> Detection::singleDetect(){
}

void Detection::detectBoard(){
    //realsenseで写真取ってopencvで矩形認識 -> 得られたpixelをさらにrealsenseで三次元座標に
    //https://github.com/opencv/opencv/blob/master/samples/cpp/squares.cpp <- 矩形認識
    //https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/cv-helpers.hpp <- opencvとrealsenseの連携
    //https://github.com/IntelRealSense/librealsense/tree/master/examples/measure  <- pixelを三次元座標に変換
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
        
        cv::pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
        cv::pyrUp(pyr, timg, image.size());
        std::vector<std::vector<cv::Point> > contours;
        
        
        for( int c = 0; c < 3; c++ )
        {
            int ch[] = {c, 0};
            cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);
            
            // try several threshold levels
            for( int l = 0; l < N; l++ )
            {
                Canny(gray0, gray, 0, thresh, 5);
                dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
                findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
                std::vector<cv::Point> approx;
                
                for( size_t i = 0; i < contours.size(); i++ )
                {
                    cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true)*0.02, true);
                    if( approx.size() == 4 &&
                    std::fabs(contourArea(approx)) > 1000 &&
                    cv::isContourConvex(approx) )
                    {
                        double maxCosine = 0;

                        for( int j = 2; j < 5; j++ )
                        {
                            double cosine = std::fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                            maxCosine = MAX(maxCosine, cosine);
                        }

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
    auto  drawSquares = []( cv::Mat& image,const std::vector<std::vector<cv::Point> >& squares )
    {
        for( size_t i = 0; i < squares.size(); i++ )
        {
            const cv::Point* p = &squares[i][0];
            int n = (int)squares[i].size();
            polylines(image, &p, &n, 1, true, cv::Scalar(0,255,0), 3, cv::LINE_AA);
        }

        imshow("TestSquares", image);
    };
    auto Distance = [](std::tuple<float, float, float> a, std::tuple<float, float, float> b){
            float x1,y1,z1,x2,y2,z2;
            x1 = std::get<0>(a);
            y1 = std::get<1>(a);
            z1 = std::get<2>(a);
            x2 = std::get<0>(b);
            y2 = std::get<1>(b);
            z2 = std::get<2>(b);
            return std::sqrt(std::pow(x1-x2,2) + std::pow(y1-y2, 2) + std::pow(z1-z2, 2));
    };
    bool is_dispersion = true;
    do {
        BoardPos.clear();
        std::vector<std::vector<cv::Point> > squares;

        rs2::colorizer color_map;
        rs2::rates_printer printer;

        do {
            auto im = pipe.wait_for_frames().get_color_frame();
            cv::Mat image = frame_to_mat(im);
        
            std::cout<<"capture"<<std::endl;
            if( image.empty() ){
                std::cerr << "Couldn't load " << std::endl;
                std::abort();
            }

            findSquares(image, squares);
            drawSquares(image, squares);
        
            int c = cv::waitKey();
        }while(squares.empty());

    
        std::vector<std::pair<int, int>> _frame_pos;
        for(auto a : squares){
            for(auto b : a)_frame_pos.push_back(std::make_pair(b.x, b.y));
        }
        std::sort(_frame_pos.begin(), _frame_pos.end());
        std::vector<std::pair<int, int>> frame_pos = { std::make_pair(0, 0), std::make_pair(0, 0), std::make_pair(0, 0), std::make_pair(0, 0)};
        for(int i = 0;i < _frame_pos.size();i++){
            if(i < _frame_pos.size()/4){
                frame_pos.at(0).first += _frame_pos.at(i).first;
                frame_pos.at(0).second += _frame_pos.at(i).second;
            }else if (i < _frame_pos.size()/2){
                frame_pos.at(1).first += _frame_pos.at(i).first;
                frame_pos.at(1).second += _frame_pos.at(i).second;

            }else if (i < _frame_pos.size()/4*3){
                frame_pos.at(2).first += _frame_pos.at(i).first;
                frame_pos.at(2).second += _frame_pos.at(i).second;

            }else{
                frame_pos.at(3).first += _frame_pos.at(i).first;
                frame_pos.at(3).second += _frame_pos.at(i).second;

            }
        }
        frame_pos.at(0).first /= _frame_pos.size()/4;
        frame_pos.at(0).second /= _frame_pos.size()/4;
        frame_pos.at(1).first /= _frame_pos.size()/4;
        frame_pos.at(1).second /= _frame_pos.size()/4;
        frame_pos.at(2).first /= _frame_pos.size()/4;
        frame_pos.at(2).second /= _frame_pos.size()/4;
        frame_pos.at(3).first /= _frame_pos.size()/4;
        frame_pos.at(3).second /= _frame_pos.size()/4;
    
        std::cout<<"Board pos in piexl"<<std::endl;
        for(int i = 0;i < 4;i++){
            std::cout<<"("<<frame_pos.at(i).first<<" "<<frame_pos.at(i).second<<")";
        }
        std::cout<<std::endl;

        rs2::frameset pframes = pipe.wait_for_frames();
        rs2::depth_frame depth = pframes.get_depth_frame();

        rs2_intrinsics intr = pframes.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data


        for(int i = 0;i < 4;i++){
            BoardPos.push_back(translatePixelToP3Doint((float)frame_pos.at(i).first, (float)frame_pos.at(i).second, intr, depth));
            std::cout<<std::get<0>(BoardPos.back())<<" "<<std::get<1>(BoardPos.back())<<" "<<std::get<2>(BoardPos.back())<<std::endl;
        }
        int idx[] = {0, 1, 3, 2, 0};
        float dispersion = 0;
        for(int i = 0;i < 4;i++){
            float d = Distance(BoardPos.at(idx[i]), BoardPos.at(idx[i+1]));
            dispersion += pow(d-BoardEdgeLen,2);
         //   std::cout<<d<<std::endl;
        }
        dispersion /= 4;
        std::cout<<dispersion<<std::endl;
        if(dispersion < dispersion_thresh)is_dispersion = false;
    }while(is_dispersion);
}

std::tuple<float, float, float> Detection::translatePixelToP3Doint(float x, float y){
    rs2::frameset pframes = pipe.wait_for_frames();
    rs2_intrinsics intr = pframes.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    float pixel[2] = {x, y};
    float qpoint[3];
    rs2::depth_frame depth = pframes.get_depth_frame();
    rs2_deproject_pixel_to_point(qpoint, &intr, pixel, depth.get_distance(pixel[0], pixel[1]));
    
    return std::make_tuple(qpoint[0], qpoint[1], qpoint[2]);
}

std::tuple<float, float, float> Detection::translatePixelToP3Doint(float x, float y, rs2_intrinsics intr, rs2::depth_frame depth){
    float pixel[2] = {x, y};
    float qpoint[3];
    rs2_deproject_pixel_to_point(qpoint, &intr, pixel, depth.get_distance(pixel[0], pixel[1]));
    
    return std::make_tuple(qpoint[0], qpoint[1], qpoint[2]);
}