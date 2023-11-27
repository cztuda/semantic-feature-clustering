#pragma once

#define PI 3.14159265358979323846
#define STRINGIFY(s) #s
#define TO_STRING(s) STRINGIFY(s)
#include <string>
struct Constant {std::string description; double value;}; 
