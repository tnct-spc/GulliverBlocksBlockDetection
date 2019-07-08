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

std::pair<std::vector<std::tuple<int,int,int>>,std::vector<std::tuple<int,int,int>>> Detection::SingleDetect(){
    const double detect_loss = 0;
    const double dispersion_accuracy = 0;

    std::vector<std::vector<double>> current_data = getDepth();

    std::vector<std::tuple<int, int, int>> add_blocks;
    std::vector<std::tuple<int, int, int>> delete_blocks;


    for(int i = 0;i < LegoWidthNumber;i++){
        for(int j = 0;j < LegoHeightNumber;j++){
            int upper_count = 0;
            int lower_count = 0;
            int flat_count = 0;
            std::pair<int, int> current_pos = std::make_pair(i * LegoWidth, j * LegoHeight);
            double sum_z = 0;
            for(int x = 0;x < LegoWidth;x++){
                for(int y = 0;y < LegoHeight;y++){
                    int pos_x = i + x;
                    int pos_y = j + y;
                    if(abs(current_data.at(pos_x).at(pos_y) - data.at(pos_x).at(pos_y)) > detect_loss){
                        if(current_data.at(pos_x).at(pos_y) - data.at(pos_x).at(pos_y) > 0){
                            upper_count++;
                        }else{
                            lower_count++;
                        }
                    }else{
                        flat_count++;
                    }
                    sum_z += current_data.at(pos_x).at(pos_y);
                }
            }
            sum_z /= (LegoHeight * LegoWidth);

            double dispersion = 0;
            
            for(int x = 0;x < LegoWidth;x++){
                for(int y = 0;y < LegoHeight;y++){
                    int pos_x = i + x;
                    int pos_y = j + y;
                    dispersion += std::pow(current_data.at(pos_x).at(pos_y) - sum_z, 2);
                }
            }
            dispersion /= (LegoHeight * LegoWidth);

            if(dispersion_accuracy < dispersion) //分散が大きいデータは無視
                flat_count = 1e9;

            if(upper_count > lower_count && upper_count > flat_count){
                std::tuple<int, int, int> detect_block_pos = std::make_tuple(i, j, std::round((sum_z - BoardDepth) / LegoDepth)-1);
                add_blocks.push_back(detect_block_pos);
            }
            if(lower_count > upper_count && lower_count > flat_count){
                std::tuple<int, int, int> detect_block_pos = std::make_tuple(i, j, std::round((sum_z - BoardDepth) / LegoDepth));
                delete_blocks.push_back(detect_block_pos);
            }
        }
    }
    data = current_data;
    return std::make_pair(add_blocks, delete_blocks);
}