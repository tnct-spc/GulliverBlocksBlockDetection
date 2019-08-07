#include<curl/curl.h>
#include<tuple>
#include<utility>
#include<vector>

class Communication{
public:
  void postJson(std::string url ,std::pair<std::vector<std::tuple<int,int,int>>,std::vector<std::tuple<int,int,int>>> block_data);

  bool isDetection(std::string url);

private:
  static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
};
