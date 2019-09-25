#include "manager.h"

Manager::Manager(){

}

void Manager::RunSingleDetection(){
    std::cout<<"Start detection"<<std::endl;
    Detection detection;
    std::cout<<"Finish constractor"<<std::endl;
    int cnt = 0;
    while(1){
        std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int,int,int>>> detect_info = detection.detect();
        for(int i = 0;i < detect_info.first.size();i++){
            std::cout<<std::get<0>(detect_info.first.at(i))<<" "<<std::get<1>(detect_info.first.at(i))<<" "<<std::get<2>(detect_info.first.at(i))<<std::endl;
        }
        //std::cout<<std::endl;
        std::cout<<detect_info.first.size()<<": Added"<<std::endl;
        for(int i = 0;i < detect_info.second.size();i++){
            std::cout<<std::get<0>(detect_info.second.at(i))<<" "<<std::get<1>(detect_info.second.at(i))<<" "<<std::get<2>(detect_info.second.at(i))<<std::endl;
        }
        std::cout<<detect_info.second.size()<<": Deleted"<<std::endl;
        std::string url = "http://gulliverblocks.herokuapp.com/add_blocks/1bfdd6d5-8d95-49c4-ad6f-a6a2a700b438/";
        Poster.postJson(url, detect_info);
        std::cout<<cnt<<std::endl;
        cnt++;
    }
    
}

void Manager::RunTestCurl(){
    std::string url = "https://gulliverblocks.herokuapp.com/add_blocks/1bfdd6d5-8d95-49c4-ad6f-a6a2a700b438/";
    std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int, int, int>>> vec;
    vec = std::make_pair(std::vector<std::tuple<int, int, int>>({std::make_tuple(0, 2, 5), std::make_tuple(0, 2, 6), std::make_tuple(0, 2, 7), std::make_tuple(0, 2, 8), std::make_tuple(0, 2, 9), std::make_tuple(0, 2, 10), std::make_tuple(0, 2, 11), std::make_tuple(0, 2, 12), std::make_tuple(1, 2, 4), std::make_tuple(1, 2, 5), std::make_tuple(1, 2, 6), std::make_tuple(1, 1, 9), std::make_tuple(1, 2, 10), std::make_tuple(2, 1, 3), std::make_tuple(2, 2, 4), std::make_tuple(2, 2, 6), std::make_tuple(2, 2, 8), std::make_tuple(3, 2, 3), std::make_tuple(3, 1, 4), std::make_tuple(3, 1, 6), std::make_tuple(3, 1, 7), std::make_tuple(4, 1, 3), std::make_tuple(4, 1, 4)}), std::vector<std::tuple<int, int, int>>({}));


    Poster.postJson(url, vec);
}
