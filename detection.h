#include <librealsense2/rs.hpp> 
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

class Detection{
private:
    int DetectNum;
    std::vector<std::vector<double>> data;

public:
    std::pair<std::pair<int, int>, std::pair<int, int>> BoardPos;
    const double LegoDepth = 0.01;
    const int LegoWidthNumber = 48;
    const int LegoHeightNumber = 42;
    const double LegoWidth = (BoardPos.second.first - BoardPos.first.first) / LegoWidthNumber;
    const double LegoHeight = (BoardPos.second.second - BoardPos.first.second) / LegoHeightNumber;
    const double BoardDepth = 0.002;

    Detection();
    std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int, int, int>>>  singleDetect(); //Widrh, Height, Depth
    std::vector<std::vector<double>> getDepth();
    std::pair<std::pair<int, int>, std::pair<int, int>> detectBoard();
    rs2::pipeline pipe;
};
