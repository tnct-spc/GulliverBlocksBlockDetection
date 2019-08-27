#include "manager.h"

Manager::Manager(){

}

void Manager::RunSingleDetection(){
    std::cout<<"Start detection"<<std::endl;
    Detection detection;
    std::cout<<"Finish constractor"<<std::endl;
    //std::set<std::tuple<int, int, int>> now_blocks;
    int now_block_num = 0;
    int cnt = 0;
    while(1){
        //std::cout<<"start detection"<<std::endl;
        std::pair<std::vector<std::tuple<int, int, int>>, std::vector<std::tuple<int,int,int>>> detect_info = detection.singleDetect();
        //std::cout<<"finish detection"<<std::endl;
        for(int i = 0;i < detect_info.first.size();i++){
            std::cout<<std::get<0>(detect_info.first.at(i))<<" "<<std::get<1>(detect_info.first.at(i))<<" "<<std::get<2>(detect_info.first.at(i))<<std::endl;
            //now_blocks.insert(std::make_tuple(std::get<0>(detect_info.first.at(i)), std::get<1>(detect_info.first.at(i)), std::get<2>(detect_info.first.at(i))));
            now_block_num++;
        }
        //std::cout<<std::endl;
        std::cout<<detect_info.first.size()<<": Added"<<std::endl;
        for(int i = 0;i < detect_info.second.size();i++){
            now_block_num--;
            std::cout<<std::get<0>(detect_info.second.at(i))<<" "<<std::get<1>(detect_info.second.at(i))<<" "<<std::get<2>(detect_info.second.at(i))<<std::endl;
            //now_blocks.erase(std::make_tuple(std::get<0>(detect_info.second.at(i)), std::get<1>(detect_info.second.at(i)), std::get<2>(detect_info.second.at(i)) + 1));
        }
     //   std::cout<<std::endl;
        std::cout<<detect_info.second.size()<<": Deleted"<<std::endl;
       // for(auto a : now_blocks){
       //     std::cout<<std::get<0>(a)<<" "<<std::get<1>(a)<<" "<<std::get<2>(a)<<std::endl;
       // }
       // std::cout<<std::endl;
        std::cout<<cnt<<std::endl;
        cnt++;
    }
    
}
