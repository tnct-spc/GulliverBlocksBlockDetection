#include "detection.h"

Detection::Detection(){
    pipe.start();
    BoardPos = detectBoard();
    data = getDepth();
}

std::vector<std::vector<double>> Detection::getDepth(){

}

std::pair<std::vector<std::tuple<int,int,int>>,std::vector<std::tuple<int,int,int>>> Detection::singleDetect(){
}

std::pair<std::pair<int, int>, std::pair<int, int>> Detection::detectBoard(){

}