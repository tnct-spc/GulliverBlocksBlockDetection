#include "detection.h"

//for now
const std::pair<std::pair<int, int>, std::pair<int, int>> Detection::BoardPos = std::make_pair(std::make_pair(0, 0), std::make_pair(0, 0));
const float Detection::LegoDepth = 0;
const float Detection::LegoWidth = 0;
const float Detection::LegoHeight = 0;

Detection::Detection(int i = 1){

    int BoardWidth = Detection::BoardPos.second.first - Detection::BoardPos.first.first;
    int BoardHeight = Detection::BoardPos.second.second - Detection::BoardPos.first.second;

    rs2::pipeline p;
    rs2::frameset frames = p.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();

    field = std::vector<std::vector<float>>(BoardHeight, std::vector<float>(BoardWidth));

    for(int i = Detection::BoardPos.first.first;i < Detection::BoardPos.second.first;i++){
        for(int j = Detection::BoardPos.first.second;j < Detection::BoardPos.second.second;j++){
            field.at(i - Detection::BoardPos.first.first).at(j - Detection::BoardPos.first.second) = depth.get_distance(i, j);
        }
    }
}

std::vector<std::tuple<int,int,int>> Detection::SingleDetect(){
    
}
