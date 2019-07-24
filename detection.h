#include <librealsense2/rs.hpp> 
#include <librealsense2/rsutil.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#pragma once

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <exception>


#include <vector>
#include <utility>
#include <cmath>
#include <iostream>
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>              
#include <iostream>             
#include <sstream>

class Detection{
private:
    int DetectNum;
    std::vector<std::vector<double>> data;

public:
    std::vector<std::tuple<float, float, float>> BoardPos;

    Detection();
    std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int, int, int>>>  singleDetect(); //Widrh, Height, Depth
    std::vector<std::vector<double>> getDepth();
    void detectBoard();
    std::tuple<float, float, float> translatePixelToP3Doint(float x, float y);
    std::tuple<float, float, float> translatePixelToP3Doint(float x, float y, rs2_intrinsics intr, rs2::depth_frame depth);

    rs2::pipeline pipe;
};