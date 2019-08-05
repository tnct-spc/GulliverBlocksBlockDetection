#include<curl/curl.h>
#include<tuple>
#include<utility>
#include<vector>
#include<ctime>


class Communication{
public:
  void postJson(std::string url ,std::pair<std::vector<std::tuple<int,int,int>>,std::vector<std::tuple<int,int,int>>> block_data);

  bool isUrl(std::string url);

private:
  static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
  {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
  }
};
