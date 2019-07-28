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

class Detection{
private:
    std::vector<std::vector<double>> data;
    const float BoardEdgeLen = 0.38; //Board Edge Length
    const float BlockEdgeLen = 0.38/17;
    const float dispersion_thresh = 0.0001;
    const int BoardEdgeNum = 17;
    const float BlockHigh = 0.009;

public:
    std::vector<std::tuple<float, float, float>> BoardPosBasedData;
    std::vector<std::pair<float, float>> BoardPos;
    std::vector<std::vector<float>> current_data;
    rs2::pipeline pipe;

    Detection();
    std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int, int, int>>>  singleDetect(); //Widrh, Height, Depth
    std::vector<std::tuple<float, float, float>> getDepth();
    void detectBoard();
    std::tuple<float, float, float> translatePixelToP3Doint(float x, float y);
    std::tuple<float, float, float> translatePixelToP3Doint(float x, float y, rs2_intrinsics intr, rs2::depth_frame depth);

    std::tuple<float, float, float> translatePlanePoint(float x, float y, float z);
    std::tuple<float, float, float> translatePlanePoint(std::tuple<float, float, float> point_pos);
};