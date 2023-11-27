
#include "functionLibSerialization.h"
#include "legacy_unserialization.h"


using namespace ofc;

functionLib::Function* functionLib::legacy::unserializeFunction(std::string type, std::istream& s){
    functionLib::Function* f;
    if(!type.compare("(polyn")){
        size_t dimY;
        size_t degree;
        s >> dimY;
        s >> degree;
        f = ofc::functionLib::legacy::unserialize_polynom(s, dimY, degree);
    }
    else if(!type.compare("(npoly")){
        size_t dimY;
        size_t degree;
        s >> dimY;
        s >> degree;
        f = ofc::functionLib::legacy::unserialize_normalizedPolynom(s, dimY, degree);
    }
    else if(!type.compare("(pwfun")){
        size_t dimY;
        s >> dimY;
        f = ofc::functionLib::legacy::unserialize_piecewiseFunction(s, dimY);
    }
    else if(!type.compare("(funct")){
        f = ofc::functionLib::legacy::unserialize_function(s);
    }
    else {
        
    }
    std::string tmp;
    s >> tmp; // remove closeing label
    return f;
}

functionLib::Function* functionLib::legacy::unserializeFunction(std::istream& s){
    std::string type;
    s >> type;
    return functionLib::legacy::unserializeFunction(type, s);
}

