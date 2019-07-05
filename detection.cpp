#include "detection.h"

//for now
const std::pair<std::pair<int, int>, std::pair<int, int>> Detection::BoardPos = std::make_pair(std::make_pair(0, 0), std::make_pair(0, 0));
const double Detection::LegoDepth = 0;
const double Detection::LegoWidth = (Detection::BoardPos.second.first - Detection::BoardPos.first.first) / Detection::LegoWitdhNumber;
const double Detection::LegoHeight = (Detection::BoardPos.second.second - Detection::BoardPos.first.second) / Detection::LegoHeightNumber;
const int Detection::LegoWidthNumber = 1;
const int Detection::LegoHeightNumber = 1;
const double Detection::BoardDepth = 1;

Detection::Detection(int i = 1){

    int BoardWidth = Detection::BoardPos.second.first - Detection::BoardPos.first.first;
    int BoardHeight = Detection::BoardPos.second.second - Detection::BoardPos.first.second;

    rs2::pipeline p;
    rs2::frameset frames = p.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();

    data = getDepth();
}

std::vector<std::vector<double>> Detection::getDepth(){
    std::vector<std::vector<double>> depth_data(BoardHeight, std::vector<double>(BoardWidth));

    for(int i = Detection::BoardPos.first.first;i < Detection::BoardPos.second.first;i++){
        for(int j = Detection::BoardPos.first.second;j < Detection::BoardPos.second.second;j++){
            depth_data.at(i - Detection::BoardPos.first.first).at(j - Detection::BoardPos.first.second) = depth.get_distance(i, j);
        }
    }
    return depth_data;
}

std::vector<std::tuple<int,int,int>> Detection::SingleDetect(){
    const double detect_loss = 0;
    const double detect_piexl = 0;
    std::vector<std::vector<double>> current_data = getDepth();

    std::vector<std::tuple<int, int, int>> detect_blocks;

    for(int i = 0;i < LegoWidthNumber;i++){
        for(int j = 0;j < LegoHeigthNumber;j++){
            int tcount = 0;
            std::pair<int, int> current_pos = std::make_pair(i * LegoWitdh, j * LegoHeight);
            double sum_z = 0;
            for(int x = 0;x < LegoWitdh;x++){
                for(int y = 0;y < LegoHeight;y++){
                    int pos_x = i + x;
                    int pos_y = j + y;
                    if(abs(current_data.at(pos_x).at(pos_y) - data.at(pos_x).at(pos_y) < detect_loss){
                        tcount++;
                        sum_z += current_data.at(pos_x).at(pos_y);
                    }
                }
            }
            if(tcount > detect_piexl){
                std::tuple detect_block_pos = std::make_tuple(i, j, std::round((sum_z - BoardDepth) / LegoDepth));
                detect_blocks.push_back(detect_block_pos);
            }
        }
    }
    return detect_blocks;
}