#include "detection.h"



Detection::Detection(int i = 1){
    data = getDepth();
}

std::vector<std::vector<double>> Detection::getDepth(){

    int BoardWidth = BoardPos.second.first - BoardPos.first.first;
    int BoardHeight = BoardPos.second.second - BoardPos.first.second;
    
    rs2::pipeline p;
    rs2::frameset frames = p.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();


    std::vector<std::vector<double>> depth_data(BoardHeight, std::vector<double>(BoardWidth));

    for(int i = BoardPos.first.first;i < BoardPos.second.first;i++){
        for(int j = BoardPos.first.second;j < BoardPos.second.second;j++){
            depth_data.at(i - BoardPos.first.first).at(j - BoardPos.first.second) = depth.get_distance(i, j);
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
        for(int j = 0;j < LegoHeightNumber;j++){
            int tcount = 0;
            std::pair<int, int> current_pos = std::make_pair(i * LegoWidth, j * LegoHeight);
            double sum_z = 0;
            for(int x = 0;x < LegoWidth;x++){
                for(int y = 0;y < LegoHeight;y++){
                    int pos_x = i + x;
                    int pos_y = j + y;
                    if(abs(current_data.at(pos_x).at(pos_y) - data.at(pos_x).at(pos_y) < detect_loss)){
                        tcount++;
                        sum_z += current_data.at(pos_x).at(pos_y);
                    }
                }
            }
            if(tcount > detect_piexl){
                std::tuple<int, int, int> detect_block_pos = std::make_tuple(i, j, std::round((sum_z - BoardDepth) / LegoDepth));
                detect_blocks.push_back(detect_block_pos);
            }
        }
    }
    data = current_data;
    return detect_blocks;
}