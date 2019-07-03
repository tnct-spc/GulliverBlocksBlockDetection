#include <librealsense2/rs.hpp> 
#include <vector>
#include <utility>

class Detection{
private:
    int DetectNum;
    std::vector<std::vector<float>> field;

public:
    static const std::pair<std::pair<float, float>, std::pair<float, float>> BoardPos;
    static const float LegoDepth;
    static const float LegoWidth;
    static const float LegoHeight;

    Detection(int i);
    std::vector<std::tuple<int,int,int>> SingleDetect(); //Widrh, Height, Depth
};
