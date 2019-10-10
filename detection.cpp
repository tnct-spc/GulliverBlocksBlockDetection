#include "detection.h"

Detection::Detection()
{

    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 6);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 6);

    pipe.start(cfg);

    detectBoard();
    based_data = std::vector<std::vector<double>>(BoardEdgeNum, std::vector<double>(BoardEdgeNum, 0));
    std::vector<std::vector<std::vector<float>>> data = std::vector<std::vector<std::vector<float>>>(BoardEdgeNum, std::vector<std::vector<float>>(BoardEdgeNum));
    field = std::vector<std::vector<std::set<std::pair<int, int>>>>(BoardEdgeNum, std::vector<std::set<std::pair<int, int>>>(BoardEdgeNum));
    float x0 = BoardPosBasedData.at(0).x;
    float y0 = BoardPosBasedData.at(0).y;
    float z0 = BoardPosBasedData.at(0).z;
    float x1 = BoardPosBasedData.at(1).x - BoardPosBasedData.at(0).x;
    float y1 = BoardPosBasedData.at(1).y - BoardPosBasedData.at(0).y;
    float z1 = BoardPosBasedData.at(1).z - BoardPosBasedData.at(0).z;
    float x2 = BoardPosBasedData.at(2).x - BoardPosBasedData.at(0).x;
    float y2 = BoardPosBasedData.at(2).y - BoardPosBasedData.at(0).y;
    float z2 = BoardPosBasedData.at(2).z - BoardPosBasedData.at(0).z;

    calc_x0 = x0;
    calc_x1 = x1;
    calc_x2 = x2;
    calc_y0 = y0;
    calc_y1 = y1;
    calc_y2 = y2;
    calc_z0 = z0;
    calc_z1 = z1;
    calc_z2 = z2;

    distance_A = std::sqrt(x1 * x1 + y1 * y1 + z1 * z1);
    distance_B = std::sqrt(x2 * x2 + y2 * y2 + z2 * z2);

    float3tuple outer = outer_product(std::make_tuple(x1, y1, z1), std::make_tuple(x2, y2, z2));

    outer_x = outer.x;
    outer_y = outer.y;
    outer_z = outer.z;

    for (int i = 0; i < 6; i++)
    {
        auto uku = getDepthAndColor();
        for (auto d : uku)
        {
            float x = d.first.x;
            float y = d.first.y;
            float z = d.first.z;
            if (0.0 < x && x < BoardEdgeLen && 0.0 < y && y < BoardEdgeLen)
            {
                if (std::floor(x / BlockEdgeLen) >= BoardEdgeNum || std::floor(y / BlockEdgeLen) >= BoardEdgeNum || std::floor(y / BlockEdgeLen) < 0 || std::floor(x / BlockEdgeLen) < 0)
                {
                    std::cerr << "detection.cpp コンストラクタ内で配列外参照だよ！！" << std::floor(x / BoardEdgeLen) << " " << std::floor(y / BoardEdgeLen) << std::endl;
                    //std::abort();
                    continue;
                }
                if (!(0.2 < x / BlockEdgeLen - std::floor(x / BlockEdgeLen) && x / BlockEdgeLen - std::floor(x / BlockEdgeLen) < 0.8))
                    continue; //あまり境界に近くないほうがよい
                if (!(0.2 < y / BlockEdgeLen - std::floor(y / BlockEdgeLen) && y / BlockEdgeLen - std::floor(y / BlockEdgeLen) < 0.8))
                    continue; //あまり境界に近くないほうがよい
                data.at(std::floor(x / BlockEdgeLen)).at(std::floor(y / BlockEdgeLen)).push_back(z);
            }
        }
    }
    cv::Mat M(960, 960, CV_8UC3, cv::Scalar(0,0,0));
    std::vector<float> depth_lists;
    for (int i = 0; i < BoardEdgeNum; i++)
    {
        for (int j = 0; j < BoardEdgeNum; j++)
        {
            std::vector<float> dis_data = data.at(i).at(j);
            if(dis_data.empty()){
                std::cout<<"認識できてないぞい"<<std::endl;
                std::abort();
            }
            float sum = std::accumulate(dis_data.begin(), dis_data.end(), 0.0);

            float average = sum / dis_data.size();

            float bunsan = 0;
            for (float a : dis_data)
            {
                bunsan += std::pow(average - a, 2);
            }
            bunsan /= dis_data.size();
            float hensa = std::sqrt(bunsan);
            float t_average = 0;
            int addcnt = 0;
            for (auto x : dis_data)
            {
                if (((x - average) / hensa) < 2)
                {
                    t_average += x;
                    addcnt++;
                }
            }
            //average = t_average / addcnt;
            std::sort(dis_data.begin(), dis_data.end());
            based_data.at(i).at(j) = dis_data.at(dis_data.size() / 2);
            //based_data.at(i).at(j) = 0;

            depth_lists.push_back(based_data.at(i).at(j));

            for(int p = i*20 ; p < 20+i*20 ; p++){
                cv::Vec3b* ptr = M.ptr<cv::Vec3b>( p );
                    for(int q = j*20 ; q < 20+j*20 ; q++){

                    ptr[q] = cv::Vec3b(based_data.at(i).at(j) * 10000 + 50, based_data.at(i).at(j) * 10000 + 50, based_data.at(i).at(j) * 10000+ 50);
                    //ptr[q] = cv::Vec3b(bunsan, bunsan, bunsan);
                }
            }
        }
    }
    std::sort(depth_lists.begin(), depth_lists.end());
    //std::reverse(depth_lists.begin(), depth_lists.end());
    for(int i = 0;i < 10;i++){
        std::cout<<i<<" "<<depth_lists.at(i)<<std::endl;
    }
    cv::imshow("Visualizer", M);
    int c = cv::waitKey();
}

std::vector<float3tuple> Detection::getDepth()
{
    std::cout << "start get Depth" << std::endl;
    float width = 1280;
    float height = 720;
    std::vector<float3tuple> depth_data;
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);
    rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth = aligned_frames.get_depth_frame();

    rs2_intrinsics intr = frames.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    for (float x = 1; x < width; x++)
    {
        for (float y = 1; y < height; y++)
        {
            depth_data.push_back(translatePlanePoint(translatePixelToP3DPoint(x, y, intr, depth)));
        }
    }
    std::cout << "finish get Depth" << std::endl;
    return depth_data;
}

std::vector<std::pair<float3tuple, std::tuple<int, int, int>>> Detection::getDepthAndColor()
{
    auto frame_to_mat = [](const rs2::frame &f) {
        auto vf = f.as<rs2::video_frame>();
        const int w = vf.get_width();
        const int h = vf.get_height();

        if (f.get_profile().format() == RS2_FORMAT_BGR8)
        {
            return cv::Mat(cv::Size(w, h), CV_8UC3, (void *)f.get_data(), cv::Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_RGB8)
        {
            auto r = cv::Mat(cv::Size(w, h), CV_8UC3, (void *)f.get_data(), cv::Mat::AUTO_STEP);
            cv::cvtColor(r, r, cv::COLOR_RGB2BGR);
            return r;
        }
        else if (f.get_profile().format() == RS2_FORMAT_Z16)
        {
            return cv::Mat(cv::Size(w, h), CV_16UC1, (void *)f.get_data(), cv::Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_Y8)
        {
            return cv::Mat(cv::Size(w, h), CV_8UC1, (void *)f.get_data(), cv::Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
        {
            return cv::Mat(cv::Size(w, h), CV_32FC1, (void *)f.get_data(), cv::Mat::AUTO_STEP);
        }

        throw std::runtime_error("Frame format is not supported yet!");
    };

    float width = 1280;
    float height = 720;
    std::vector<std::pair<float3tuple, std::tuple<int, int, int>>> data;
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);
    rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth = aligned_frames.get_depth_frame();

    const double scale = 0.001;

    auto z_pixels = reinterpret_cast<const uint16_t *>(depth.get_data());

    rs2_intrinsics intr = frames.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    cv::Mat image = frame_to_mat(color_frame);

    float pixel[2];
    float qpoint[3];
    float3tuple pos;
    int idx;
    for (int y = 1; y <= height; y++)
    {
        cv::Vec3b *ptr = image.ptr<cv::Vec3b>(y - 1);
        for (int x = 1; x <= width; x++)
        {
            pixel[0] = x;
            pixel[1] = y;
            idx = (x - 1) + (y - 1) * width;
            float z_distance = (int)(z_pixels[idx]) * scale;
            rs2_deproject_pixel_to_point(qpoint, &intr, pixel, z_distance);

            cv::Vec3b bgr = ptr[x - 1];
            pos = translatePlanePoint(qpoint[0], qpoint[1], qpoint[2]);
            if (!(0.0 <= pos.x && pos.x < BoardEdgeLen && 0.0 <= pos.y && pos.y < BoardEdgeLen))
            {
                continue;
            }
            //data.push_back(std::make_pair(translatePlanePoint(qpoint[0], qpoint[1], qpoint[2]), std::tuple<int, int, int>(bgr[2], bgr[1], bgr[0]))); //bgr to rgb //めっちゃpush_backすると遅いのでいっぺんに確保してしまう
            data.emplace_back(std::make_pair(pos, std::make_tuple(bgr[2], bgr[1], bgr[0]))); //bgr to rgb 実験用
        }
    }
    return data;
}

std::pair<std::vector<std::pair<std::tuple<int, int, int>, int>>, std::vector<std::tuple<int, int, int>>> Detection::singleDetect()
{
    std::chrono::system_clock::time_point start, end;
    start = std::chrono::system_clock::now();

    int frame_num = 2;
    std::vector<std::vector<std::vector<std::vector<float>>>> multiframe_data = std::vector<std::vector<std::vector<std::vector<float>>>>(BoardEdgeNum, std::vector<std::vector<std::vector<float>>>(BoardEdgeNum, std::vector<std::vector<float>>(frame_num, std::vector<float>({}))));
    //std::vector<std::vector<bool>> hand_flag(BoardEdgeNum, std::vector<bool>(BoardEdgeNum, true)); //手の判定
    std::vector<std::vector<std::vector<float>>> data = std::vector<std::vector<std::vector<float>>>(BoardEdgeNum, std::vector<std::vector<float>>(BoardEdgeNum, std::vector<float>({})));
    std::vector<std::vector<std::vector<int>>> RGB_R = std::vector<std::vector<std::vector<int>>>(BoardEdgeNum, std::vector<std::vector<int>>(BoardEdgeNum, std::vector<int>({})));
    std::vector<std::vector<std::vector<int>>> RGB_G = std::vector<std::vector<std::vector<int>>>(BoardEdgeNum, std::vector<std::vector<int>>(BoardEdgeNum, std::vector<int>({})));
    std::vector<std::vector<std::vector<int>>> RGB_B = std::vector<std::vector<std::vector<int>>>(BoardEdgeNum, std::vector<std::vector<int>>(BoardEdgeNum, std::vector<int>({})));

    std::cout << "getting data now" << std::endl;
    for (int i = 0; i < frame_num; i++)
    {
        std::vector<std::pair<float3tuple, std::tuple<int, int, int>>> depth_data = getDepthAndColor();
        for (auto d : depth_data)
        {
            float x = d.first.x;
            float y = d.first.y;
            float z = d.first.z;
            int r = std::get<0>(d.second);
            int g = std::get<1>(d.second);
            int b = std::get<2>(d.second);

            if (0.0 <= x && x < BoardEdgeLen && 0.0 <= y && y < BoardEdgeLen)
            {
                if (std::floor(x / BlockEdgeLen) >= BoardEdgeNum || std::floor(y / BlockEdgeLen) >= BoardEdgeNum || std::floor(y / BlockEdgeLen) < 0 || std::floor(x / BlockEdgeLen) < 0)
                {
                    std::cerr << "detection.cpp getDepth関数内で配列外参照だよ！！" << std::floor(x / BlockEdgeLen) << " " << std::floor(y / BlockEdgeLen) << std::endl;
                    //std::abort();
                    continue;
                }

                if (!(0.3 < x / BlockEdgeLen - std::floor(x / BlockEdgeLen) && x / BlockEdgeLen - std::floor(x / BlockEdgeLen) < 0.7))
                    continue; //あまり境界に近くないほうがよい
                if (!(0.3 < y / BlockEdgeLen - std::floor(y / BlockEdgeLen) && y / BlockEdgeLen - std::floor(y / BlockEdgeLen) < 0.7))
                    continue; //あまり境界に近くないほうがよい
                data.at(std::floor(x / BlockEdgeLen)).at(std::floor(y / BlockEdgeLen)).emplace_back(z);
                //multiframe_data.at(std::floor(x / BlockEdgeLen)).at(std::floor(y / BlockEdgeLen)).at(i).emplace_back(z);
                RGB_R.at(std::floor(x / BlockEdgeLen)).at(std::floor(y / BlockEdgeLen)).emplace_back(r);
                RGB_G.at(std::floor(x / BlockEdgeLen)).at(std::floor(y / BlockEdgeLen)).emplace_back(g);
                RGB_B.at(std::floor(x / BlockEdgeLen)).at(std::floor(y / BlockEdgeLen)).emplace_back(b);
            }
        }
    }
    std::cout << "finish get data" << std::endl;

    //cv::Mat M(960, 960, CV_8UC3, cv::Scalar(0,0,0));
    std::cout << std::setprecision(3);
    std::vector<float> depth_data_list;
    std::vector<std::pair<int, int>> color_distance_list;
    std::vector<int> data_num_lists;
    std::vector<std::pair<std::tuple<int, int, int>, int>> add;
    std::vector<std::tuple<int, int, int>> remove;
    for (int i = 0; i < BoardEdgeNum; i++)
    {
        for (int j = 0; j < BoardEdgeNum; j++)
        {
            //      if(!hand_flag.at(i).at(j))continue;
            /*
            for(int p = i*20 ; p < 20+i*20 ; p++){
                cv::Vec3b* ptr = M.ptr<cv::Vec3b>( p );
                for(int q = j*20 ; q < 20+j*20 ; q++){

                    //ptr[q] = cv::Vec3b(median * BlockHigh * 5000 + 50, median * BlockHigh * 5000 + 50, median * BlockHigh * 5000+ 50);
                    //ptr[q] = cv::Vec3b(data.at(i).at(j).size(), data.at(i).at(j).size(), data.at(i).at(j).size());
                    }
            }
            */

            
            std::vector<float> grid_data = data.at(i).at(j);

            data_num_lists.push_back(data.at(i).at(j).size());

            if (data.at(i).at(j).size() <= 15)
                continue;

            std::vector<int> grid_data_r = RGB_R.at(i).at(j);
            std::vector<int> grid_data_g = RGB_G.at(i).at(j);
            std::vector<int> grid_data_b = RGB_B.at(i).at(j);

            int r = grid_data_r.at(grid_data_r.size() / 2);
            int g = grid_data_g.at(grid_data_g.size() / 2);
            int b = grid_data_b.at(grid_data_b.size() / 2);

            int color = 0;
            int color_distance = 1e9;
            for (int k = 0; k < BlockColors.size(); k++)
            {
                if (std::pow(r - std::get<0>(BlockColors.at(k)), 2) + std::pow(g - std::get<1>(BlockColors.at(k)), 2) + std::pow(b - std::get<2>(BlockColors.at(k)), 2) < color_distance)
                {
                    color_distance = 2 * std::pow(r - std::get<0>(BlockColors.at(k)), 2) + 4 * std::pow(g - std::get<1>(BlockColors.at(k)), 2) + 3 * std::pow(b - std::get<2>(BlockColors.at(k)), 2);
                    color = k;
                }
            }
            
           
            color_distance_list.push_back({color_distance, color});

            std::sort(grid_data.begin(), grid_data.end());
            float median = grid_data.at(grid_data.size() / 2);

            median -= based_data.at(i).at(j);
            median /= BlockHigh;
            depth_data_list.push_back(median);
            int high = std::round(median);
            high = std::max(high, 0);
            if(high >= 6)continue;
            while (true)
            {
                auto itr = field.at(i).at(j).upper_bound({high, 1e9});
                if (itr == field.at(i).at(j).end())
                    break;
                auto a = *itr;
                remove.push_back(std::make_tuple(i, a.first, j)); //x z y
                field.at(i).at(j).erase(itr);
            }
             if (color_distance > 5000)
                continue;

            if(color == BlockColors.size() - 1)color = 12;

            auto a = *field.at(i).at(j).upper_bound({high, -1});
            if(a.first == high && high != 0 && a.second != color){
                remove.push_back(std::make_tuple(i, high, j));
                add.push_back(std::make_pair(std::make_tuple(i, high, j), color));
                field.at(i).at(j).insert({high, color});
                field.at(i).at(j).erase(a);
            }

            if (high != 0 && field.at(i).at(j).find({high, color}) == field.at(i).at(j).end())
            {
                field.at(i).at(j).insert({high, color});
                add.push_back(std::make_pair(std::make_tuple(i, high, j), color)); //x z y
            }


            if (std::isnan(median))
            {
                std::cout << "average is nan" << std::endl;
            }
            
           
            
        }
    }
    std::sort(data_num_lists.begin(), data_num_lists.end());
    std::sort(depth_data_list.begin(), depth_data_list.end());
    std::sort(color_distance_list.begin(), color_distance_list.end());
    std::reverse(depth_data_list.begin(), depth_data_list.end());
    
    std::cout << "depth" << std::endl;
    for (int i = 0; i < std::min(10, (int)depth_data_list.size()); i++)
    {
        std::cout << i << "th : " << depth_data_list.at(i) << std::endl;
    }
    std::cout << "color" << std::endl;
    for (int i = 0; i < std::min(10, (int)color_distance_list.size()); i++)
    {
        std::cout << i << "th : " << color_distance_list.at(i).first << " " << color_distance_list.at(i).second << std::endl;
    }
    std::cout<<"data num"<<std::endl;
    for(int i = 0;i < std::min(10, (int)data_num_lists.size()); i++){
        std::cout << i << "th : " << data_num_lists[i] << std::endl;
    }
    //cv::imshow("Visualizer", M);
    //int c = cv::waitKey();
    end = std::chrono::system_clock::now();
    double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0);
    std::cout << time / 1000 << "s" << std::endl;

    return std::make_pair(add, remove);
}

void Detection::detectBoard()
{
    //realsenseで写真取ってopencvで矩形認識 -> 得られたpixelをさらにrealsenseで三次元座標に
    //https://github.com/opencv/opencv/blob/master/samples/cpp/squares.cpp <- 矩形認識
    //https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/cv-helpers.hpp <- opencvとrealsenseの連携
    //https://github.com/IntelRealSense/librealsense/tree/master/examples/measure  <- pixelを三次元座標に変換
    auto angle = [](cv::Point pt1, cv::Point pt2, cv::Point pt0) {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
    };

    auto findSquares = [&](const cv::Mat &image, std::vector<std::vector<cv::Point>> &squares) {
        int thresh = 1200, N = 11;
        squares.clear();

        cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;

        cv::pyrDown(image, pyr, cv::Size(image.cols / 2, image.rows / 2));
        cv::pyrUp(pyr, timg, image.size());
        std::vector<std::vector<cv::Point>> contours;

        for (int c = 0; c < 3; c++)
        {
            int ch[] = {c, 0};
            cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);

            for (int l = 0; l < N; l++)
            {
                Canny(gray0, gray, 0, thresh, 5);
                dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));
                findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
                std::vector<cv::Point> approx;

                for (size_t i = 0; i < contours.size(); i++)
                {
                    cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true) * 0.02, true);
                    if (approx.size() == 4 &&
                        std::fabs(contourArea(approx)) > 1000 &&
                        cv::isContourConvex(approx))
                    {
                        double maxCosine = 0;

                        for (int j = 2; j < 5; j++)
                        {
                            double cosine = std::fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                            maxCosine = MAX(maxCosine, cosine);
                        }

                        if (maxCosine < 0.3)
                            squares.push_back(approx);
                    }
                }
            }
        }
    };
    auto frame_to_mat = [](const rs2::frame &f) {
        auto vf = f.as<rs2::video_frame>();
        const int w = vf.get_width();
        const int h = vf.get_height();

        if (f.get_profile().format() == RS2_FORMAT_BGR8)
        {
            return cv::Mat(cv::Size(w, h), CV_8UC3, (void *)f.get_data(), cv::Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_RGB8)
        {
            auto r = cv::Mat(cv::Size(w, h), CV_8UC3, (void *)f.get_data(), cv::Mat::AUTO_STEP);
            cv::cvtColor(r, r, cv::COLOR_RGB2BGR);
            return r;
        }
        else if (f.get_profile().format() == RS2_FORMAT_Z16)
        {
            return cv::Mat(cv::Size(w, h), CV_16UC1, (void *)f.get_data(), cv::Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_Y8)
        {
            return cv::Mat(cv::Size(w, h), CV_8UC1, (void *)f.get_data(), cv::Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
        {
            return cv::Mat(cv::Size(w, h), CV_32FC1, (void *)f.get_data(), cv::Mat::AUTO_STEP);
        }

        throw std::runtime_error("Frame format is not supported yet!");
    };
    auto drawSquares = [](cv::Mat &image, const std::vector<std::vector<cv::Point>> &squares) {
        for (size_t i = 0; i < squares.size(); i++)
        {
            const cv::Point *p = &squares[i][0];
            int n = (int)squares[i].size();
            polylines(image, &p, &n, 1, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        }

            imshow("TestSquares", image);
            int c = cv::waitKey();
    };
    auto Distance = [](float3tuple a, float3tuple b) {
        float x1, y1, z1, x2, y2, z2;
        x1 = a.x;
        y1 = a.y;
        z1 = a.z;
        x2 = b.x;
        y2 = b.y;
        z2 = b.z;
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) + std::pow(z1 - z2, 2));
    };

    int frame_num = 3;
    BoardPosBasedData = std::vector<float3tuple>(4);

    for (int frame_idx = 0; frame_idx < frame_num; frame_idx++)
    {
        bool is_dispersion = true;
        std::vector<float3tuple> ProvisionalBoardPosBasedData;
        do
        {
            std::vector<std::vector<cv::Point>> squares;

            rs2::colorizer color_map;
            rs2::rates_printer printer;

            do
            {
                auto im = pipe.wait_for_frames().get_color_frame();
                // std::cout<<"BBB"<<std::endl;
                cv::Mat image = frame_to_mat(im);
                std::cout << image.rows << " " << image.cols << std::endl;

                std::cout << "capture" << std::endl;
                if (image.empty())
                {
                    std::cerr << "Couldn't load " << std::endl;
                    std::abort();
                }
                /*
                findSquares(image, squares);
                drawSquares(image, squares);
                */
                cv::imshow("www", image);
                cv::setMouseCallback("www", mouse_callback);

            } while (squares.empty());

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
            int area_idx = 0;
            for (auto a : squares)
            {
                int area = 0;
                for (int i = 0; i < 4; i++)
                {
                    area += ((a.at(i).x - a.at((i + 1) % 4).x) * (a.at(i).y + a.at((i + 1) % 4).y));
                }
                area = std::abs(area) / 2;
                frame_list.push_back({area, area_idx});
                area_idx++;
                std::cout << area << std::endl;
            }
            sort(frame_list.begin(), frame_list.end());
            int frame_idx = std::upper_bound(frame_list.begin(), frame_list.end(), std::make_pair(1.0 * 1e4 * 5, -1)) - frame_list.begin();
            if (frame_idx == frame_list.size()){
                continue;
                //よくない
            }
            /*
            for(; frame_idx < frame_list.size(); frame_idx++){
            }
            */
            for (int i = 0; i < 4; i++)
            {
                frame_pos.push_back({squares.at(frame_list.at(frame_idx).second).at(i).x, squares.at(frame_list.at(frame_idx).second).at(i).y});
            }

            std::sort(frame_pos.begin(), frame_pos.end());
            std::cout << "Board pos in piexl" << std::endl;
            for (int i = 0; i < 4; i++)
            {
                std::cout << "(" << frame_pos.at(i).first << " " << frame_pos.at(i).second << ")";
            }
            std::cout << std::endl;
            std::sort(frame_pos.begin(), frame_pos.end());

            rs2::frameset pframes = pipe.wait_for_frames();
            rs2::align align(RS2_STREAM_COLOR);
            auto aligned_frames = align.process(pframes);
            rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
            rs2::depth_frame depth = aligned_frames.get_depth_frame();

            std::cout << depth.get_width() << " " << depth.get_height() << std::endl;

            rs2_intrinsics intr = pframes.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data

            for (int i = 0; i < 4; i++)
            {
                ProvisionalBoardPosBasedData.push_back(translatePixelToP3DPoint((float)frame_pos.at(i).first, (float)frame_pos.at(i).second, intr, depth)); // なんかこの返り値が 0 0 0 になるケースがある。
                std::cout << ProvisionalBoardPosBasedData.back().x << " " << ProvisionalBoardPosBasedData.back().y << " " << ProvisionalBoardPosBasedData.back().z << std::endl;
            }
            std::vector<std::pair<float, int>> dist_idx_pair;
            for (int i = 0; i < 4; i++)
            {
                dist_idx_pair.push_back(std::make_pair(Distance(ProvisionalBoardPosBasedData.at(i), ProvisionalBoardPosBasedData.at(0)), i));
            }
            std::vector<float3tuple> _BoardPosBasedData;
            std::sort(dist_idx_pair.begin(), dist_idx_pair.end());
            for (int i = 0; i < 4; i++)
            {
                _BoardPosBasedData.push_back(ProvisionalBoardPosBasedData.at(dist_idx_pair.at(i).second));
                std::cout<<dist_idx_pair.at(i).first<<std::endl;
            }
            ProvisionalBoardPosBasedData = _BoardPosBasedData;
            int idx[] = {0, 1, 3, 2, 0};
            float dispersion = 0;
            float edge_average = 0;
            for (int i = 0; i < 4; i++)
            {
                float d = Distance(ProvisionalBoardPosBasedData.at(idx[i]), ProvisionalBoardPosBasedData.at(idx[i + 1]));
                dispersion += std::pow(d - BoardEdgeLen, 2);
                edge_average += d;
                std::cout << d << std::endl;
            }
            edge_average /= 4;
            dispersion /= 4;
            std::cout << dispersion << std::endl;
            if (dispersion < dispersion_thresh)
            {
                is_dispersion = false;
                BoardEdgeLen = edge_average;
                BlockEdgeLen = BoardEdgeLen / BoardEdgeNum;
            }
            else
            {
                std::cout << "Board pos in piexl" << std::endl;
                for (int i = 0; i < 4; i++)
                {
                    std::cout << "(" << ProvisionalBoardPosBasedData[i].x << " " << ProvisionalBoardPosBasedData[i].y << " " << ProvisionalBoardPosBasedData[i].y << ")";
                }
                std::cout << std::endl;
            }
        } while (is_dispersion);

        std::vector<std::pair<float, int>> dist_idx_pair;
        for (int i = 0; i < 4; i++)
        {
            dist_idx_pair.push_back(std::make_pair(Distance(ProvisionalBoardPosBasedData.at(i), float3tuple(-1e6, -1e6, -1e6)), i));
        }
        auto q = *std::min_element(dist_idx_pair.begin(), dist_idx_pair.end());
        int _i = q.second;
        std::swap(ProvisionalBoardPosBasedData.at(_i), ProvisionalBoardPosBasedData.at(0));
        std::vector<float3tuple> _BoardPosBasedData;
        dist_idx_pair.clear();
        for (int i = 0; i < 4; i++)
        {
            dist_idx_pair.push_back(std::make_pair(Distance(ProvisionalBoardPosBasedData.at(i), ProvisionalBoardPosBasedData.at(0)), i));
        }
        std::sort(dist_idx_pair.begin(), dist_idx_pair.end());
        for (int i = 0; i < 4; i++)
        {
            _BoardPosBasedData.push_back(ProvisionalBoardPosBasedData.at(dist_idx_pair.at(i).second));
        }
        if (_BoardPosBasedData.at(1).x > _BoardPosBasedData.at(2).x)
        { //ここ本質で、x座標が小さいほうがy軸によるように調整している(そうしないと右手系左手系のごちゃごちゃで、z座標がひっくり返ることがあるため)
            std::swap(_BoardPosBasedData.at(1), _BoardPosBasedData.at(2));
        }
        ProvisionalBoardPosBasedData = _BoardPosBasedData;
        for(int j = 0;j < 4;j++){
            BoardPosBasedData.at(j).x += ProvisionalBoardPosBasedData.at(j).x;
            BoardPosBasedData.at(j).y += ProvisionalBoardPosBasedData.at(j).y;
            BoardPosBasedData.at(j).z += ProvisionalBoardPosBasedData.at(j).z;
        }
    }
    for(int i = 0;i < 4;i++){
        BoardPosBasedData.at(i).x /= frame_num;
        BoardPosBasedData.at(i).y /= frame_num;
        BoardPosBasedData.at(i).z /= frame_num;
    }
}

float3tuple Detection::translatePixelToP3DPoint(float x, float y)
{
    rs2::frameset pframes = pipe.wait_for_frames();
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(pframes);
    rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth = aligned_frames.get_depth_frame();

    rs2_intrinsics intr = pframes.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    float pixel[2] = {x, y};
    float qpoint[3];

    rs2_deproject_pixel_to_point(qpoint, &intr, pixel, rs2_depth_frame_get_distance(depth.get(), pixel[0], pixel[1], &e));

    return float3tuple(qpoint[0], qpoint[1], qpoint[2]);
}

float3tuple Detection::translatePixelToP3DPoint(float x, float y, rs2_intrinsics &intr, rs2::depth_frame &depth)
{
    pixel[0] = x;
    pixel[1] = y;

    rs2_error *e = nullptr;
    rs2_deproject_pixel_to_point(qpoint, &intr, pixel, rs2_depth_frame_get_distance(depth.get(), pixel[0], pixel[1], &e));

    return float3tuple(qpoint[0], qpoint[1], qpoint[2]);
}

float3tuple Detection::translatePlanePoint(float x, float y, float z)
{

    x -= calc_x0;
    y -= calc_y0;
    z -= calc_z0;

    return float3tuple((x * calc_x1 + y * calc_y1 + z * calc_z1) / distance_A, (x * calc_x2 + y * calc_y2 + z * calc_z2) / distance_B, (x * outer_x + y * outer_y + z * outer_z) / distance_A / distance_B);
}

float3tuple Detection::translatePlanePoint(float3tuple V)
{

    V.x -= calc_x0;
    V.y -= calc_y0;
    V.z -= calc_z0;

    return float3tuple((V.x * calc_x1 + V.y * calc_y1 + V.z * calc_z1) / distance_A, (V.x * calc_x2 + V.y * calc_y2 + V.z * calc_z2) / distance_B, (V.x * outer_x + V.y * outer_y + V.z * outer_z) / distance_A / distance_B);
}
float Detection::inner_product(std::tuple<float, float, float> a, std::tuple<float, float, float> b)
{
    return std::get<0>(a) * std::get<0>(b) + std::get<1>(a) * std::get<1>(b) + std::get<2>(a) * std::get<2>(b);
}
float3tuple Detection::outer_product(std::tuple<float, float, float> a, std::tuple<float, float, float> b)
{
    return float3tuple(std::get<1>(a) * std::get<2>(b) - std::get<2>(a) * std::get<1>(b), std::get<2>(a) * std::get<0>(b) - std::get<0>(a) * std::get<2>(b), std::get<0>(a) * std::get<1>(b) - std::get<1>(a) * std::get<0>(b));
}
void mouse_callback(int event, int x, int y, int flags, void *userdata, std::vector<cv::Point> squares)
{
    if (event == cv::EVENT_MOUSEMOVE) {
        std::cout << "(" << x << ", " << y << ")" << std::endl;
    }
}
