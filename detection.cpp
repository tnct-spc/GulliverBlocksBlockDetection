#include "detection.h"

//for now
const std::pair<std::pair<float, float>, std::pair<float, float>> Detection::BoardPos = std::make_pair(std::make_pair(0, 0), std::make_pair(0, 0));
const float Detection::LegoDepth = 0;
const float Detection::LegoWidth = 0;
const float Detection::LegoHeight = 0;

Detection::Detection(int i = 1){

}

std::vector<std::tuple<int,int,int>> Detection::SingleDetect(){

} 
