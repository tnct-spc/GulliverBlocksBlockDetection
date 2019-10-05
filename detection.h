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
    const double dispersion_thresh = 0.0005;
    const int BoardEdgeNum = 48;
    const double BlockHigh = 0.009;
    const float BlockHighthresh = 0.5;

    const std::vector<std::tuple<int, int, int>> BlockColors = { std::make_tuple(50, 50, 50), std::make_tuple(201, 26, 9), std::make_tuple(242, 205, 55), std::make_tuple(254, 138, 24), std::make_tuple(187, 232, 11), std::make_tuple(159, 195, 233), std::make_tuple(0, 85, 191), std::make_tuple(35, 120, 65), std::make_tuple(146, 57, 120), std::make_tuple(187, 233, 11) };

public:
    std::vector<std::vector<std::tuple<double, double, double>>> MultiBoardPosBasedData;
    std::vector<std::vector<std::set<int>>> field;
    std::vector<std::vector<double>> based_data;
    std::vector<rs2::pipeline> pipelines;
    rs2::context ctx;
    rs2::config cfg;

    std::vector<double> distance_A;
    std::vector<double> distance_B;
    std::vector<double> calc_x0;
    std::vector<double> calc_x1;
    std::vector<double> calc_x2;
    std::vector<double> calc_y0;
    std::vector<double> calc_y1;
    std::vector<double> calc_y2;
    std::vector<double> calc_z0;
    std::vector<double> calc_z1;
    std::vector<double> calc_z2;
    std::vector<double> outer_x;
    std::vector<double> outer_y;
    std::vector<double> outer_z;

    const int sensor_num = 2;


    Detection();
    std::pair<std::vector<std::pair<std::tuple<int, int, int>, int>>, std::vector<std::tuple<int, int, int>>>  singleDetect(); //Widrh, Height, Depth
    std::vector<std::tuple<float, float, float>> getDepth();
    std::vector<std::pair<std::tuple<float, float, float>, std::tuple<int, int, int>>> getDepthAndColor();

    void detectBoard();
    std::tuple<float, float, float> translatePixelToP3DPoint(float x, float y);
    std::tuple<float, float, float> translatePixelToP3DPoint(float x, float y, rs2_intrinsics& intr, rs2::depth_frame& depth);

    std::tuple<float, float, float> translatePlanePoint(float x, float y, float z, int sensor_num);
    std::tuple<float, float, float> translatePlanePoint(std::tuple<float, float, float> point_pos, int sensor_num);

    float inner_product(std::tuple<float, float, float> a, std::tuple<float, float, float> b);
    std::tuple<float, float, float> outer_product(std::tuple<float, float, float> a, std::tuple<float, float, float> b);

 

};
