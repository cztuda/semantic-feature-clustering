
#include "elzinga_grid.h"

#include <iostream>
#include <string>
#include <vector>

using namespace std;

int main(){
    vector<int> x(6), y(6);
    x[0] = 1; y[0] = 6;
    x[1] = 2; y[1] = 5;
    x[2] = 3; y[2] = 4; 
    x[3] = 4; y[3] = 3;
    x[4] = 5; y[4] = 2;
    x[5] = 6; y[5] = 1;
    
    SVRspellOptions options;
    
    double xx, yy, xy;
    double dist = distance(x, y, options, xx, yy, xy);
    std::cout << "dist=" << dist << endl;
    std::cout << "xx=" << xx << endl;
    std::cout << "yy=" << yy << endl;
    std::cout << "xy=" << xy << endl;
    
    cout << "finished." << endl;
}
