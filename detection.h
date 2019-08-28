#include <librealsense2/rs.hpp> 
#include <librealsense2/rsutil.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <opencv2/opencv.hpp>
#include <exception>


#include <vector>
#include <utility>
#include <cmath>
#include <iostream>
#include <math.h>
#include <tuple>
#include <limits>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <thread>

class Detection{
private:
    std::vector<std::vector<double>> data;
    float BoardEdgeLen = 0.385; //Board Edge Length
    float BlockEdgeLen = 0.385 / 48.0;
    const float dispersion_thresh = 0.0005;
    const int BoardEdgeNum = 48;
    const float BlockHigh = 0.009;
    const float BlockHighthresh = 0.5;

public:
    std::vector<std::tuple<float, float, float>> BoardPosBasedData;
    std::vector<std::vector<std::set<int>>> field;
    std::vector<std::vector<float>> based_data;
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile profile;
    rs2::device dev;

    std::vector<std::thread> threads;
    int cpu_num;

    Detection();
    std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int, int, int>>>  singleDetect(); //Widrh, Height, Depth
    std::vector<std::tuple<float, float, float>> getDepth();
    std::vector<std::tuple<float, float, float>> getDepthFaster();

    void detectBoard();
    std::tuple<float, float, float> translatePixelToP3DPoint(float x, float y);
    std::tuple<float, float, float> translatePixelToP3DPoint(float x, float y, rs2_intrinsics& intr, rs2::depth_frame& depth);


    std::tuple<float, float, float> translatePlanePoint(float x, float y, float z);
    std::tuple<float, float, float> translatePlanePoint(std::tuple<float, float, float> point_pos);
};