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
    const double LegoDepth = 0.01;
    const int LegoWidthNumber = 48;
    const int LegoHeightNumber = 42;

    const double BoardDepth = 0.002;

    Detection();
    std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int, int, int>>>  singleDetect(); //Widrh, Height, Depth
    std::vector<std::vector<double>> getDepth();
    void detectBoard();
    rs2::pipeline pipe;
};
