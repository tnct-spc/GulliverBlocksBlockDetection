#include <librealsense2/rs.hpp> 
#include <vector>
#include <utility>

class Detection{
private:
    int DetectNum;
    std::vector<std::vector<float>> field;
    const std::pair<std::pair<float, float>, std::pair<float, float>> BoardPos;
public:
    Detection(int i);
    std::vector<std::vector<float>> SingleDetect();
};
