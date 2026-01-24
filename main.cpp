#include "detector/GreenLightDetector.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
    GreenLightDetector detector;
    
    // 配置文件路径
    string config_path = "../config/config.yaml";
    if (argc > 1) {
        config_path = argv[1];
        cout << "Using config file: " << config_path << endl;
    } else {
        cout << "Using default config file: ../config/config.yaml" << endl;
    }
    
    if (!detector.initialize(config_path)) {
        cerr << "Initialization failed!" << endl;
        return 1;
    }
    
    try {
        detector.run();
    }
    catch (const exception& e) {
        cerr << "ERROR: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}