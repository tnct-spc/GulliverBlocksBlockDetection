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
#include <chrono>

struct float3tuple //std::tupleは遅い
{
    float x,y,z;
    float3tuple(){
        x = 0;
        y = 0;
        z = 0;
    }
    float3tuple(float a, float b, float c){
        x = a;
        y = b;
        z = c;
    }
    
};


class Detection{
private:
    std::vector<std::vector<double>> data;
    //float BoardEdgeLen = 0.385; //Board Edge Length
    //float BlockEdgeLen = 0.385 / 48.0;
    float BoardEdgeLen = 0.280;
    float BlockEdgeLen = 0.280 / 35;
    const double dispersion_thresh = 0.0005;
    //const int BoardEdgeNum = 48;
    const int BoardEdgeNum = 35;
    const double BlockHigh = 0.009;
    const float BlockHighthresh = 0.5;

    const std::vector<std::tuple<int, int, int>> BlockColors = { std::make_tuple(10, 26, 35), std::make_tuple(134, 18, 26), std::make_tuple(175, 154, 30), std::make_tuple(1e3, 1e3, 1e3), std::make_tuple(1e3, 1e3, 1e3), std::make_tuple(1e3, 1e3, 1e3), std::make_tuple(0, 62, 146), std::make_tuple(0, 73, 58), std::make_tuple(1e3, 1e3, 1e3), std::make_tuple(137, 182, 208) };

public:
    std::vector<float3tuple> BoardPosBasedData;
    std::vector<std::vector<std::set<std::pair<int, int>>>> field;
    std::vector<std::vector<double>> based_data;
    rs2::pipeline pipe;
    rs2::config cfg;

    float distance_A;
    float distance_B;
    float calc_x0;
    float calc_x1;
    float calc_x2;
    float calc_y0;
    float calc_y1;
    float calc_y2;
    float calc_z0;
    float calc_z1;
    float calc_z2;
    float outer_x;
    float outer_y;
    float outer_z;
    rs2_error * e = nullptr;
    float pixel[2];
    float qpoint[3];


    Detection();

    std::pair<std::vector<std::pair<std::tuple<int, int, int>, int>>, std::vector<std::tuple<int, int, int>>>  singleDetect(); //Widrh, Height, Depth
    std::vector<float3tuple> getDepth();
    std::vector<std::pair<float3tuple, std::tuple<int, int, int>>> getDepthAndColor();

    void detectBoard();
    float3tuple translatePixelToP3DPoint(float x, float y);
    float3tuple translatePixelToP3DPoint(float x, float y, rs2_intrinsics& intr, rs2::depth_frame& depth);

    float3tuple translatePlanePoint(float x, float y, float z);
    float3tuple translatePlanePoint(float3tuple point_pos);

    float inner_product(std::tuple<float, float, float> a, std::tuple<float, float, float> b);
    float3tuple outer_product(std::tuple<float, float, float> a, std::tuple<float, float, float> b);

   
};

