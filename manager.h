#include "detection.h"
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <ctime>
#include <stdlib.h> // defines putenv in POSIX



class Manager{

public:
    Manager();
    void RunSingleDetection();
    //void RunTestCurl();
    Communication Poster;
};
