#include "manager.h"

Manager::Manager(){

}

void Manager::RunSingleDetection(){
    std::cout<<"Start detection"<<std::endl;
    Detection detection;
    std::cout<<"Finish constractor"<<std::endl;
    /* 
    while(1){
        std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int,int,int>>> detect_info = detection.singleDetect();
        for(int i = 0;i < detect_info.first.size();i++){
            std::cout<<std::get<0>(detect_info.first.at(i))<<" "<<std::get<1>(detect_info.first.at(i))<<" "<<std::get<2>(detect_info.first.at(i))<<std::endl;
        }
        std::cout<<std::endl;
        std::cout<<detect_info.first.size()<<": Added"<<std::endl;
        for(int i = 0;i < detect_info.second.size();i++){
            std::cout<<std::get<0>(detect_info.second.at(i))<<" "<<std::get<1>(detect_info.second.at(i))<<" "<<std::get<2>(detect_info.second.at(i))<<std::endl;
        }
        std::cout<<std::endl;
        std::cout<<detect_info.second.size()<<": Deleted"<<std::endl;
    }
    */
}
