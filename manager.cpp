#include "manager.h"

Manager::Manager(){

}

void Manager::RunSingleDetection(){
    Detection detection;
    while(1){
        std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int,int,int>>> detect_info = detection.SingleDetect();
        for(int i = 0;i < detect_info.first.size();i++){
            std::cout<<std::get<0>(detect_info.first.at(i))<<" "<<std::get<1>(detect_info.first.at(i))<<" "<<std::get<2>(detect_info.first.at(i))<<std::endl;
        }
        std::cout<<"Added"<<std::endl;
        for(int i = 0;i < detect_info.second.size();i++){
            std::cout<<std::get<0>(detect_info.second.at(i))<<" "<<std::get<1>(detect_info.second.at(i))<<" "<<std::get<2>(detect_info.second.at(i))<<std::endl;
        }
    }
}
