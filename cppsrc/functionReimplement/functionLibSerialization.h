#pragma once

#include <string>
#include <iostream>

#include "Function.h"

namespace ofc {
    namespace functionLib {
        namespace legacy {
            
            Function* unserializeFunction(std::istream& s);
            Function* unserializeFunction(std::string type, std::istream& s);
            
        }
    }
}
