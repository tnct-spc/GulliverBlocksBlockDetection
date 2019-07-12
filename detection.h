#include <librealsense2/rs.hpp> 
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>

class Detection{
private:
    int DetectNum;
    std::vector<std::vector<double>> data;

public:
    //for now
    const std::pair<std::pair<int, int>, std::pair<int, int>> BoardPos = std::make_pair(std::make_pair(168, 0), std::make_pair(541, 347));
    const double LegoDepth = 0.01;
    const int LegoWidthNumber = 48;
    const int LegoHeightNumber = 42;
    const double LegoWidth = (BoardPos.second.first - BoardPos.first.first) / LegoWidthNumber;
    const double LegoHeight = (BoardPos.second.second - BoardPos.first.second) / LegoHeightNumber;
    const double BoardDepth = 0.002;

    Detection();
    std::pair<std::vector<std::tuple<int,int,int>>, std::vector<std::tuple<int,int,int>>>  SingleDetect(); //Widrh, Height, Depth
    std::vector<std::vector<double>> getDepth();
    rs2::pipeline p;
};
