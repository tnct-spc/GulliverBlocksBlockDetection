#include"communication.h"

void Communication::postJson(std::string url ,std::pair<std::vector<std::tuple<int,int,int>>,std::vector<std::tuple<int,int,int>>> block_data){
  int id = 0;
  int color_id = 0;
  std::string put;
  std::time_t unix_time = std::time(nullptr);
  int x,y,z;
  
  //ブロックの設置か削除かを確認
  if (
      std::get<0>(block_data.first.at(0)) == 0 &&
      std::get<1>(block_data.first.at(0)) == 0 &&
      std::get<2>(block_data.first.at(0)) == 0
      )
  {
      x = std::get<0>(block_data.second.at(0));
      y = std::get<1>(block_data.second.at(0));
      z = std::get<2>(block_data.second.at(0));

      put = "false";
  } else {
      x = std::get<0>(block_data.first.at(0));
      y = std::get<1>(block_data.first.at(0));
      z = std::get<2>(block_data.first.at(0));

      put = "true";
  }
  
  
  std::string json = "{\"blocks\":[{\"ID\":"  + std::to_string(id) \
                   + ",\"colorID\":" + std::to_string(color_id) \
                   + ",\"put\":" + put \
                   + ",\"time\":" + std::to_string(unix_time) \
                   + ",\"x\":" + std::to_string(x) \
                   + ",\"y\":" + std::to_string(y) \
                   + ",\"z\":" + std::to_string(z) \
                   + "}]}";

  //サーバーにPOST
  CURL *hnd;

  hnd = curl_easy_init();
  curl_easy_setopt(hnd, CURLOPT_URL, url.c_str());
  curl_easy_setopt(hnd, CURLOPT_POSTFIELDS, json.c_str());
  curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "POST");

  curl_easy_perform(hnd);
  curl_easy_cleanup(hnd);
  hnd = NULL;

}

bool Communication::isUrl(std::string url){
  CURL *hnd;
  std::string readBuffer;

  hnd = curl_easy_init();
  curl_easy_setopt(hnd, CURLOPT_URL, url.c_str());
  curl_easy_setopt(hnd, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(hnd, CURLOPT_WRITEDATA, &readBuffer);

  curl_easy_perform(hnd);
  curl_easy_cleanup(hnd);
  hnd = NULL;

  if (readBuffer == "1") 
    return true;
  else if (readBuffer == "0") 
    return false;
  
}