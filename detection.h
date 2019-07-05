#include <librealsense2/rs.hpp> 
#include <vector>
#include <utility>
#include <cmath>

class Detection{
private:
    int DetectNum;
    std::vector<std::vector<double>> data;

public:
    //for now
    const std::pair<std::pair<int, int>, std::pair<int, int>> BoardPos = std::make_pair(std::make_pair(0, 0), std::make_pair(0, 0));
    const double LegoDepth = 0;
    const int LegoWidthNumber = 1;
    const int LegoHeightNumber = 1;
    const double LegoWidth = (BoardPos.second.first - BoardPos.first.first) / LegoWidthNumber;
    const double LegoHeight = (BoardPos.second.second - BoardPos.first.second) / LegoHeightNumber;
    const double BoardDepth = 1;

    Detection(int i);
    std::vector<std::tuple<int,int,int>> SingleDetect(); //Widrh, Height, Depth
    std::vector<std::vector<double>> getDepth();
};
