#include "detection.h"

Detection::Detection(){
    //test_opencv();
    cpu_num = std::thread::hardware_concurrency();

    threads.resize(cpu_num);    

    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    profile = pipe.start(cfg);
    dev = profile.get_device();

    detectBoard();
    based_data = std::vector<std::vector<float>>(BoardEdgeNum, std::vector<float>(BoardEdgeNum, 0));
    std::vector<std::vector<std::vector<float>>> data = std::vector<std::vector<std::vector<float>>>(BoardEdgeNum, std::vector<std::vector<float>>(BoardEdgeNum));
    field = std::vector<std::vector<std::set<int>>>(BoardEdgeNum, std::vector<std::set<int>>(BoardEdgeNum));
    float x1 = std::get<0>(BoardPosBasedData.at(1)) - std::get<0>(BoardPosBasedData.at(0));
    float y1 = std::get<1>(BoardPosBasedData.at(1)) - std::get<1>(BoardPosBasedData.at(0));
    float z1 = std::get<2>(BoardPosBasedData.at(1)) - std::get<2>(BoardPosBasedData.at(0));
    float x2 = std::get<0>(BoardPosBasedData.at(2)) - std::get<0>(BoardPosBasedData.at(0));
    float y2 = std::get<1>(BoardPosBasedData.at(2)) - std::get<1>(BoardPosBasedData.at(0));
    float z2 = std::get<2>(BoardPosBasedData.at(2)) - std::get<2>(BoardPosBasedData.at(0));

    calc_x1 = x1;
    calc_x2 = x2;
    calc_y1 = y1;
    calc_y2 = y2;
    calc_z1 = z1;
    calc_z2 = z2;
    
    distance_A = std::sqrt(x1 * x1 + y1 * y1 + z1 * z1);
    distance_B = std::sqrt(x2 * x2 + y2 * y2 + z2 * z2);

    std::tuple<float, float, float> outer = outer_product(std::make_tuple(x1, y1, z1), std::make_tuple(x2, y2, z2));

    outer_x = std::get<0>(outer);
    outer_y = std::get<1>(outer);
    outer_z = std::get<2>(outer);


    for(int i = 0;i < 3;i++){
        auto uku = getDepthAndColor();
        for(auto d : uku){
            float x = std::get<0>(d.first);
            float y = std::get<1>(d.first);
            float z = std::get<2>(d.first);
            if(0.0 < x && x < BoardEdgeLen && 0.0 < y && y < BoardEdgeLen){
                if(x / BlockEdgeLen >= BoardEdgeNum || y / BlockEdgeLen >= BoardEdgeNum || y / BlockEdgeLen < 0 || x / BlockEdgeLen < 0){
                    std::cerr<<"detection.cpp コンストラクタ内で配列外参照だよ！！"<< std::floor(x / BoardEdgeLen) << " "<<std::floor(y / BoardEdgeLen)<<std::endl;
                    std::abort();
                }
                if(!(0.1 < x / BlockEdgeLen - std::floor(x / BlockEdgeLen) && x / BlockEdgeLen - std::floor(x / BlockEdgeLen) < 0.9))continue; //あまり境界に近くないほうがよい
                if(!(0.1 < y / BlockEdgeLen - std::floor(y / BlockEdgeLen) && y / BlockEdgeLen - std::floor(y / BlockEdgeLen) < 0.9))continue; //あまり境界に近くないほうがよい
                data.at(std::floor(x / BlockEdgeLen)).at(std::floor(y / BlockEdgeLen)).push_back(z);
            }
        }
    }

    std::cout<<"got data"<<std::endl;
    for(int i = 0;i < BoardEdgeNum;i++){
        for(int j = 0;j < BoardEdgeNum;j++){
            //std::cout<<data.at(i).at(j).size()<<std::endl;
            std::vector<float> dis_data = data.at(i).at(j);
            float sum = std::accumulate(dis_data.begin(), dis_data.end(), 0.0); 

            float average = sum / dis_data.size();

            float bunsan = 0;
            for(float a : dis_data){
                bunsan += std::pow(average - a, 2);
            }
            bunsan /= dis_data.size();
            float hensa = std::sqrt(bunsan);
            float t_average = 0;
            int addcnt = 0;
            for(auto x : dis_data){
                if(((x - average) / hensa) < 2){
                    t_average += x;
                    addcnt++;
                }
            }
            average = t_average / addcnt;
            based_data.at(i).at(j) = average;
        }
    }
    
}

std::vector<std::tuple<float, float, float>> Detection::getDepth(){
    std::cout<<"get Depth"<<std::endl;
    float width = 1280;
    float height = 720;
    std::vector<std::tuple<float, float, float>> depth_data;
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);
    rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth = aligned_frames.get_depth_frame();

    rs2_intrinsics intr = frames.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    for(float x = 1;x < width;x++){
        for(float y = 1;y < height;y++){
            depth_data.push_back(translatePlanePoint(translatePixelToP3DPoint(x, y, intr, depth)));
        }
    }
    std::cout<<"finish get Depth"<<std::endl;
    return depth_data;
}

std::vector<std::pair<std::tuple<float, float, float>, std::tuple<int, int, int>>> Detection::getDepthAndColor(){

    

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

    float width = 1280;
    float height = 720;
    std::vector<std::pair<std::tuple<float, float, float>, std::tuple<int, int, int>>> data(width * height);
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);
    rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth = aligned_frames.get_depth_frame();

    rs2::sensor sensor = dev.query_sensors()[0];
    rs2::depth_sensor depth_sensor = sensor.as<rs2::depth_sensor>();

    const double scale = depth_sensor.get_depth_scale();

    auto z_pixels = reinterpret_cast<const uint16_t*>(depth.get_data());

    rs2_intrinsics intr = frames.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    cv::Mat image = frame_to_mat(color_frame);

    int cnt = 0;
    float pixel[2];
    float qpoint[3];

    for(int y = 1;y <= height;y++){
        cv::Vec3b* ptr = image.ptr<cv::Vec3b>( y-1 );
        for(int x = 1;x <= width ; x++){
            pixel[0] = x;
            pixel[1] = y;
            int idx = (x-1) + (y-1) * width;
            float z_distance = (int)(z_pixels[idx]) * scale;
            rs2_deproject_pixel_to_point(qpoint, &intr, pixel, z_distance);

            cv::Vec3b bgr = ptr[x-1];

            data[cnt] = std::make_pair(translatePlanePoint(qpoint[0], qpoint[1], qpoint[2]), std::tuple<int, int, int>(bgr[2], bgr[1], bgr[0])); //bgr to rgb
            cnt++;
            //data.emplace_back(std::make_pair(std::make_tuple(qpoint[0], qpoint[1], qpoint[2]), std::make_tuple(bgr[2], bgr[1], bgr[0]))); //bgr to rgb 実験用
            
        }
    }
    

    return data;
    

    /*
    std::vector<std::pair<std::tuple<float, float, float>, std::tuple<int, int, int>>> data;
   

    int width = 1280;
    int height = 720;
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);
    rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth = aligned_frames.get_depth_frame();

    rs2::pointcloud pc;
    rs2::points points;

    pc.map_to(color_frame);

    points = pc.calculate(depth);

    int bytes = color_frame.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = color_frame.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(color_frame.get_data());

    auto vertices = points.get_vertices();              // get vertices
    auto tex_coords = points.get_texture_coordinates(); // and texture coordinates

    for(int i = 0 ; i < points.size() ; i++){
        if(vertices[i].z){
            int Text_Index = (bytes *  std::min(std::max(int(tex_coords[i].u * width  + .5f), 0), width - 1) 
                            + strides * std::min(std::max(int(tex_coords[i].v * height + .5f), 0), height - 1));
            
            int NT1 = New_Texture[Text_Index];
            int NT2 = New_Texture[Text_Index + 1];
            int NT3 = New_Texture[Text_Index + 2];

            data.push_back(std::make_pair(translatePlanePoint(vertices[i].x, vertices[i].y, vertices[i].z), std::make_tuple(NT1, NT2, NT3)));

        }
    }
    std::cout<<data.size()<<std::endl;
    return data;
    */
}

std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int, int, int>>> Detection::singleDetect(){

    int t_num = 6;
    std::vector<std::vector<std::vector<std::vector<float>>>> multiframe_data = std::vector<std::vector<std::vector<std::vector<float>>>>(BoardEdgeNum, std::vector<std::vector<std::vector<float>>>(BoardEdgeNum, std::vector<std::vector<float>>(t_num, std::vector<float>({}))));
    std::vector<std::vector<bool>> flag(BoardEdgeNum, std::vector<bool>(BoardEdgeNum, true));
    std::vector<std::vector<std::vector<float>>> data = std::vector<std::vector<std::vector<float>>>(BoardEdgeNum, std::vector<std::vector<float>>(BoardEdgeNum, std::vector<float>({})));
    std::cout<<"getting data now"<<std::endl;
    for(int i = 0;i < t_num;i++){
        std::vector<std::pair<std::tuple<float, float, float>, std::tuple<int, int, int>>> depth_data = getDepthAndColor();
        for(auto d : depth_data){
            float x = std::get<0>(d.first);
            float y = std::get<1>(d.first);
            float z = std::get<2>(d.first);
            if(0.0 < x && x < BoardEdgeLen && 0.0 < y && y < BoardEdgeLen){
                if(x / BlockEdgeLen >= BoardEdgeNum || y / BlockEdgeLen >= BoardEdgeNum || y / BlockEdgeLen < 0.0 || x / BlockEdgeLen < 0.0){
                    std::cerr<<"detection.cpp getDepth関数内で配列外参照だよ！！"<< x / BoardEdgeLen << " "<<y / BoardEdgeLen <<std::endl;
                    std::abort();
                }
                if(!(0.2 < x / BlockEdgeLen - std::floor(x / BlockEdgeLen) && x / BlockEdgeLen - std::floor(x / BlockEdgeLen) < 0.8))continue; //あまり境界に近くないほうがよい
                if(!(0.2 < y / BlockEdgeLen - std::floor(y / BlockEdgeLen) && y / BlockEdgeLen - std::floor(y / BlockEdgeLen) < 0.8))continue; //あまり境界に近くないほうがよい
                data.at(std::floor(x / BlockEdgeLen)).at(std::floor(y / BlockEdgeLen)).emplace_back(z);
                multiframe_data.at(std::floor(x / BlockEdgeLen)).at(std::floor(y / BlockEdgeLen)).at(i).emplace_back(z);
            }
        }
    }
    std::cout<<"finish get data"<<std::endl;

    
    for(int i = 0;i < BoardEdgeNum;i++){
        for(int j = 0;j < BoardEdgeNum;j++){
            std::vector<float> front;
            std::vector<float> back;
            float front_ave;
            float back_ave;
            for(int k = 0;k < t_num/2;k++){
                for(auto x : multiframe_data.at(i).at(j).at(k))front.push_back(x);
            }
            std::vector<float> dis_data = front;
            float sum = std::accumulate(dis_data.begin(), dis_data.end(), 0.0); 

            float average = sum / dis_data.size();

            float bunsan = 0;
            for(float a : dis_data){
                bunsan += std::pow(average - a, 2);
            }
            bunsan /= dis_data.size();
            float hensa = std::sqrt(bunsan);
            float t_average = 0;
            int addcnt = 0;
            for(auto x : dis_data){
                if(((x - average) / hensa) < 2){
                    t_average += x;
                    addcnt++;
                }
            }
            front_ave = t_average / addcnt;
            for(int k = t_num;k < t_num;k++){
                for(auto x : multiframe_data.at(i).at(j).at(k))front.push_back(x);
            }

            dis_data = back;
            sum = std::accumulate(dis_data.begin(), dis_data.end(), 0.0); 

            average = sum / dis_data.size();

            bunsan = 0;
            for(float a : dis_data){
                bunsan += std::pow(average - a, 2);
            }
            bunsan /= dis_data.size();
            hensa = std::sqrt(bunsan);
            t_average = 0;
            addcnt = 0;
            for(auto x : dis_data){
                if(((x - average) / hensa) < 2){
                    t_average += x;
                    addcnt++;
                }
            }
            back_ave = t_average / addcnt;
            front_ave -= based_data.at(i).at(j);
            back_ave -= based_data.at(i).at(j);
            //前3フレームと後ろ3フレームの平均が2ブロック以上離れていたら切ってみる
            //他の案として、別々にはずれ値処理をするのではなく、全体ではずれ値処理をしてから前と後ろで平均の差を取るというものがある
            if(abs(back_ave - front_ave) / BlockHigh >= 2){
                flag.at(i).at(j) = false;
                std::cout<<"disable detect pos("<<i<<","<<j<<")"<<std::endl;
            }

        }
    }
    
    std::cout<<"uku"<<std::endl;

    cv::Mat M(960, 960, CV_8UC3, cv::Scalar(0,0,0));
    //std::cout<< std::setprecision(3);
    std::vector<float> uku;
    std::vector<std::tuple<int, int, int>> add;
    std::vector<std::tuple<int, int, int>> remove;
    for(int i = 0;i < BoardEdgeNum;i++){
        for(int j = 0;j < BoardEdgeNum;j++){
            if(!flag.at(i).at(j))continue;    
            std::vector<float> dis_data = data.at(i).at(j);
            float sum = std::accumulate(dis_data.begin(), dis_data.end(), 0.0); 

            float average = sum / dis_data.size();

            float bunsan = 0;
            for(float a : dis_data){
                bunsan += std::pow(average - a, 2);
            }
            bunsan /= dis_data.size();
            float hensa = std::sqrt(bunsan);
            float t_average = 0;
            int addcnt = 0;
            for(auto x : dis_data){
                if(((x - average) / hensa) < 2){
                    t_average += x;
                    addcnt++;
                }
            }
            //average = t_average / addcnt;
            std::sort(dis_data.begin(), dis_data.end());
            average = dis_data.at(dis_data.size() / 2);
            average -= based_data.at(i).at(j);
            average /= BlockHigh;
            uku.push_back(average);
            int high = std::round(average);
            high = std::max(high, 0);
            while(true){
                auto itr = field.at(i).at(j).upper_bound(high);
                if(itr == field.at(i).at(j).end())break;
                field.at(i).at(j).erase(itr);
                remove.push_back(std::make_tuple(i, j, high));
            }
            if(high != 0 && field.at(i).at(j).find(high) == field.at(i).at(j).end()){
                field.at(i).at(j).insert(high);
                add.push_back(std::make_tuple(i, high, j));
            }
            // f.push_back(average * BlockHigh);
            if(std::isnan(average)){
                std::cout<<"nanananana"<<std::endl;
            }
            
            for(int p = i*20 ; p < 20+i*20 ; p++){
                cv::Vec3b* ptr = M.ptr<cv::Vec3b>( p );
                for(int q = j*20 ; q < 20+j*20 ; q++){

                    ptr[q] = cv::Vec3b(average * BlockHigh *5000+100, average * BlockHigh *5000+100, average * BlockHigh *5000+100);
                    //ptr[q] = cv::Vec3b(high *5000+100, high *5000+100, high *5000+100);
                }
            }
            
            
        }
    }
    std::sort(uku.begin(), uku.end());
    std::reverse(uku.begin(), uku.end());
    for(int i = 0;i < 10;i++){
        std::cout<<i<<" : "<<uku.at(i)<<std::endl;
    }
    cv::imshow("Visualizer", M);
    int c = cv::waitKey();
    return std::make_pair(add, remove);
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
        int c = cv::waitKey();
    };
    auto Distance = [](std::tuple<float, float, float> a, std::tuple<float, float, float> b){
            float x1,y1,z1,x2,y2,z2;
            x1 = std::get<0>(a);
            y1 = std::get<1>(a);
            z1 = std::get<2>(a);
            x2 = std::get<0>(b);
            y2 = std::get<1>(b);
            z2 = std::get<2>(b);
            return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1-y2, 2) + std::pow(z1-z2, 2));
    };
    bool is_dispersion = true;
    do {
        BoardPosBasedData.clear();
        std::vector<std::vector<cv::Point> > squares;

        rs2::colorizer color_map;
        rs2::rates_printer printer;

        do {
            auto im = pipe.wait_for_frames().get_color_frame();
           // std::cout<<"BBB"<<std::endl;
            cv::Mat image = frame_to_mat(im);
            std::cout<<image.rows<<" "<<image.cols<<std::endl;
        
            std::cout<<"capture"<<std::endl;
            if( image.empty() ){
                std::cerr << "Couldn't load " << std::endl;
                std::abort();
            }
            
            //findSquares(image, squares);
            cv::imwrite("ukunikia.png", image);
            std::cout<<"please enter point pos"<<std::endl;
            std::vector<cv::Point> inputs(4);
            for(int i = 0;i < 4;i++){
                int x,y;std::cin >> x >> y;
                std::cout<<"("<<x<<","<<y<<")"<<std::endl;
                inputs.at(i).x = x;
                inputs.at(i).y = y;
            }
            squares.push_back(inputs);
            drawSquares(image, squares);
        }while(squares.empty());

        

        std::vector<std::pair<int, int>> frame_pos;
        //とりあえず面積の中央値の枠を採用することにする
        /*
        std::vector<std::pair<int, int>> _frame_pos;
        for(auto a : squares){
            for(auto b : a)_frame_pos.push_back(std::make_pair(b.x, b.y));
        }
        std::sort(_frame_pos.begin(), _frame_pos.end(),[](auto a, auto b){ std::pow(a.first, 2) + std::pow(a.second, 2) < std::pow(b.first, 2) + std::pow(b.second, 2); });
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
        //安易にソートするのは絶対に良くない、x座標が等しい時に死んでしまう可能性がある 要改善

        frame_pos.at(0).first /= _frame_pos.size()/4;
        frame_pos.at(0).second /= _frame_pos.size()/4;
        frame_pos.at(1).first /= _frame_pos.size()/4;
        frame_pos.at(1).second /= _frame_pos.size()/4;
        frame_pos.at(2).first /= _frame_pos.size()/4;
        frame_pos.at(2).second /= _frame_pos.size()/4;
        frame_pos.at(3).first /= _frame_pos.size()/4;
        frame_pos.at(3).second /= _frame_pos.size()/4;

        std::sort(frame_pos.begin(), frame_pos.end());
        */
        std::vector<std::pair<double, int>> frame_list;
        int qwerty = 0;
        for(auto a : squares){
            double ans = 0;
            for(int i = 0;i < 4;i++){
                ans += ((a.at(i).x - a.at((i+1)%4).x) * (a.at(i).y + a.at((i+1)%4).y));
            }
            ans = std::abs(ans) / 2;
            frame_list.push_back({ans, qwerty});
            qwerty++;
            std::cout<<ans<<std::endl;
        }
        sort(frame_list.begin(), frame_list.end());
        int frame_idx = std::upper_bound(frame_list.begin(), frame_list.end(), std::make_pair(1.0*1e5, 0)) - frame_list.begin();
        if(frame_idx == frame_list.size())continue;
        for(int i = 0;i < 4;i++){
            frame_pos.push_back({squares.at(frame_list.at(frame_idx).second).at(i).x, squares.at(frame_list.at(frame_idx).second).at(i).y});
        }

        std::sort(frame_pos.begin(), frame_pos.end());
        std::cout<<"Board pos in piexl"<<std::endl;
        for(int i = 0;i < 4;i++){
            std::cout<<"("<<frame_pos.at(i).first<<" "<<frame_pos.at(i).second<<")";
        }
        std::cout<<std::endl;
        std::sort(frame_pos.begin(), frame_pos.end());

        rs2::frameset pframes = pipe.wait_for_frames();
        rs2::align align(RS2_STREAM_COLOR);
        auto aligned_frames = align.process(pframes);
        rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
        rs2::depth_frame depth = aligned_frames.get_depth_frame();

        std::cout<<depth.get_width()<<" "<<depth.get_height()<<std::endl;

        rs2_intrinsics intr = pframes.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data


        for(int i = 0;i < 4;i++){
            BoardPosBasedData.push_back(translatePixelToP3DPoint((float)frame_pos.at(i).first, (float)frame_pos.at(i).second, intr, depth));
            std::cout<<std::get<0>(BoardPosBasedData.back())<<" "<<std::get<1>(BoardPosBasedData.back())<<" "<<std::get<2>(BoardPosBasedData.back())<<std::endl;
        }
        std::vector<std::pair<float, int>> ins;
        for(int i = 0;i < 4;i++){
            ins.push_back(std::make_pair(Distance(BoardPosBasedData.at(i), BoardPosBasedData.at(0)), i));
        }
        std::vector<std::tuple<float, float, float>> _BoardPosBasedData;
        std::sort(ins.begin(), ins.end());
        for(int i = 0;i < 4;i++){
            _BoardPosBasedData.push_back(BoardPosBasedData.at(ins.at(i).second));
        }
        BoardPosBasedData = _BoardPosBasedData;
        int idx[] = {0, 1, 3, 2, 0};
        float dispersion = 0;
        float edge_average = 0;
        for(int i = 0;i < 4;i++){
            float d = Distance(BoardPosBasedData.at(idx[i]), BoardPosBasedData.at(idx[i+1]));
            dispersion += std::pow(d - BoardEdgeLen, 2);
            edge_average += d;
            std::cout<<d<<std::endl;
        }
        edge_average /= 4;
        dispersion /= 4;
        std::cout<<dispersion<<std::endl;
        if(dispersion < dispersion_thresh){
            is_dispersion = false;
            BoardEdgeLen = edge_average;
            BlockEdgeLen = BoardEdgeLen / BoardEdgeNum; 
        }else{
            std::cout<<"Board pos in piexl"<<std::endl;
            for(int i = 0;i < 4;i++){
                std::cout<<"("<<std::get<0>(BoardPosBasedData[i])<<" "<<std::get<1>(BoardPosBasedData[i])<<" "<<std::get<2>(BoardPosBasedData[i])<<")";
            }
            std::cout<<std::endl;
        }
    }while(is_dispersion);

    std::vector<std::pair<float, int>> ins;
    for(int i = 0;i < 4;i++){
        ins.push_back(std::make_pair(Distance(BoardPosBasedData.at(i), std::make_tuple(0, 0, 0)), i));
    }
    auto q = *std::min_element(ins.begin(), ins.end());
    int _i = q.second;
    std::swap(BoardPosBasedData.at(_i), BoardPosBasedData.at(0));
    std::vector<std::tuple<float, float, float>> _BoardPosBasedData;
    ins.clear();
    for(int i = 0;i < 4;i++){
        ins.push_back(std::make_pair(Distance(BoardPosBasedData.at(i), BoardPosBasedData.at(0)), i));
    }
    std::sort(ins.begin(), ins.end());
    for(int i = 0;i < 4;i++){
        _BoardPosBasedData.push_back(BoardPosBasedData.at(ins.at(i).second));
    }
    if(std::get<0>(_BoardPosBasedData.at(1)) < std::get<0>(_BoardPosBasedData.at(2))){
        std::swap(_BoardPosBasedData.at(1), _BoardPosBasedData.at(2));
    }
    BoardPosBasedData = _BoardPosBasedData;
}

std::tuple<float, float, float> Detection::translatePixelToP3DPoint(float x, float y){
    rs2::frameset pframes = pipe.wait_for_frames();
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(pframes);
    rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth = aligned_frames.get_depth_frame();

    rs2_intrinsics intr = pframes.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    float pixel[2] = {x, y};
    float qpoint[3];

    rs2_error * e = nullptr;
    rs2_deproject_pixel_to_point(qpoint, &intr, pixel, rs2_depth_frame_get_distance(depth.get(), pixel[0], pixel[1], &e));
    
    return std::make_tuple(qpoint[0], qpoint[1], qpoint[2]);
}

std::tuple<float, float, float> Detection::translatePixelToP3DPoint(float x, float y, rs2_intrinsics& intr, rs2::depth_frame& depth){
    float pixel[2] = {x, y};
    float qpoint[3];

    rs2_error * e = nullptr;
    rs2_deproject_pixel_to_point(qpoint, &intr, pixel, rs2_depth_frame_get_distance(depth.get(), pixel[0], pixel[1], &e));
    
    return std::make_tuple(qpoint[0], qpoint[1], qpoint[2]);
}

std::tuple<float, float, float> Detection::translatePlanePoint(float x, float y, float z){

    x -= std::get<0>(BoardPosBasedData.at(0));
    y -= std::get<1>(BoardPosBasedData.at(0));
    z -= std::get<2>(BoardPosBasedData.at(0));

    return std::make_tuple((x * calc_x1 + y * calc_y1 + z * calc_z1) / distance_A, (x * calc_x2 + y * calc_y2 + z * calc_z2) / distance_B, (x * outer_x + y * outer_y + z * outer_z) / distance_A / distance_B);
}

std::tuple<float, float, float> Detection::translatePlanePoint(std::tuple<float, float, float> V){
    float x = std::get<0>(V);
    float y = std::get<1>(V);
    float z = std::get<2>(V);

    x -= std::get<0>(BoardPosBasedData.at(0));
    y -= std::get<1>(BoardPosBasedData.at(0));
    z -= std::get<2>(BoardPosBasedData.at(0));

    return std::make_tuple((x * calc_x1 + y * calc_y1 + z * calc_z1) / distance_A, (x * calc_x2 + y * calc_y2 + z * calc_z2) / distance_B, (x * outer_x + y * outer_y + z * outer_z) / distance_A / distance_B);
}
void Detection::test_opencv(){
     // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensor
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

        float* _depth = (float*)depth.get_data();

        for(int i = 0;;i++){
            std::cout<<i<<" : "<<_depth[i]<<std::endl;
        }

        // Update the window with new data
        imshow(window_name, image);
    }

    return;
}
float Detection::inner_product(std::tuple<float, float, float> a, std::tuple<float, float, float> b){
    return std::get<0>(a) * std::get<0>(b) + std::get<1>(a) * std::get<1>(b) + std::get<2>(a) * std::get<2>(b);
}
std::tuple<float, float, float> Detection::outer_product(std::tuple<float, float, float> a, std::tuple<float, float, float> b){
    return std::make_tuple(std::get<1>(a) * std::get<2>(b) - std::get<2>(a) * std::get<1>(b), std::get<2>(a) * std::get<0>(b) - std::get<0>(a) * std::get<2>(b), std::get<0>(a) * std::get<1>(b) - std::get<1>(a) * std::get<0>(b));
}