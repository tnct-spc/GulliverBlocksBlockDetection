#include"communication.h"

void Communication::postJson(std::string url, std::pair<std::vector<std::pair<std::tuple<int,int,int>, int>>, std::vector<std::tuple<int,int,int>>> block_data){
    std::string json = "{\"blocks\":[";
   
    if (!block_data.first.empty())
    {
        for (size_t i = 0; i < block_data.first.size(); i++){
            json = json + "{\"put\":" + "true" \
                        + ",\"colorID\":" + "\""+ std::to_string(block_data.first.at(i).second) + "\"" \
                        + ",\"x\":" + std::to_string(std::get<0>(block_data.first.at(i).first)) \
                        + ",\"y\":" + std::to_string(std::get<1>(block_data.first.at(i).first)) \
                        + ",\"z\":" + std::to_string(std::get<2>(block_data.first.at(i).first)) \
                        + "}";
            if (!(i + 1 == block_data.first.size() && block_data.second.empty()))
            {
                json += ",";
            }
      
        }

   }
   if (!block_data.second.empty())
   {
       for (size_t i = 0; i < block_data.second.size(); i++)
       {
           json = json + "{\"put\":" + "false" \
                       + ",\"x\":" + std::to_string(std::get<0>(block_data.second.at(i))) \
                       + ",\"y\":" + std::to_string(std::get<1>(block_data.second.at(i))) \
                       + ",\"z\":" + std::to_string(std::get<2>(block_data.second.at(i))) \
                       + "}";
            if (i + 1 != block_data.second.size())
            {
                json += ",";
            }
        }
    }
  
    json += "]}";

    std::cout<<json<<std::endl;
  
    //サーバーにPOST
    CURL *hnd;
    hnd = curl_easy_init();
    struct curl_slist *headers = NULL;

    headers = curl_slist_append(headers, "Content-Type: application/json;");
    headers = curl_slist_append(headers, "Accept: application/json;");
    headers = curl_slist_append(headers, "charsets: utf-8;");

    curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, headers);
  
    curl_easy_setopt(hnd, CURLOPT_URL, url.c_str());
    curl_easy_setopt(hnd, CURLOPT_POSTFIELDS, json.c_str());

    curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "POST");

    curl_easy_perform(hnd);
    curl_easy_cleanup(hnd);
    hnd = NULL;
} 

bool Communication::isDetection(std::string url){
    CURL *hnd;
    std::string readBuffer;

    hnd = curl_easy_init();
    curl_easy_setopt(hnd, CURLOPT_URL, url.c_str());
    curl_easy_setopt(hnd, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(hnd, CURLOPT_WRITEDATA, &readBuffer);

    curl_easy_perform(hnd);
    curl_easy_cleanup(hnd);
    hnd = NULL;

    return readBuffer == "1";   
}


size_t Communication::writeCallback(void *contents, size_t size, size_t nmemb, void *userp){
   ((std::string*)userp)->append((char*)contents, size * nmemb);
   return size * nmemb;
}
