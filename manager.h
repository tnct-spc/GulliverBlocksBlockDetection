#include "communication.h" 
#include "detection.h"
#include <iostream>
#include <unistd.h>


class Manager{

public:
    Manager();
    void RunSingleDetection();
    //void RunTestCurl();
    Communication Poster;
};
