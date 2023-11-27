
#include "../functionReimplement/functionLibSerialization.h"
#include "../util/onCleanup.h"

#include "../functionReimplement/NormalizedPolynom.h"
#include "../functionReimplement/PiecewiseFunction.h"

#include <fstream>

#include "mex_directives.h"
#include "class_handle.hpp"


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    
    MEX_CHECK(legacyUnserialize, 2-2, 1, -1)      
    if(!mxIsChar(prhs[0])){
        mexErrMsgTxt("Second argument (no handleptr required) must be a string.");
    }
    if(nlhs == 0){
        return;
    }
    
    std::string input;
    MEX_GET_STRING(legacyUnserialize, input, -2)
    
    double isFilename;
    MEX_GET_DOUBLE(legacyUnserialize, isFilename, -1)
    
    ofc::functionLib::Function* f = 0;
    
    if(isFilename) {
        std::ifstream stream(input);
        //ofc::onCleanup closeStream([stream](){stream.close();});
        ofc::functionLib::Function* f = 0;
        f = ofc::functionLib::legacy::unserializeFunction(stream);
        stream.close();
    }
    else {
        std::istringstream stream(input);
        f = ofc::functionLib::legacy::unserializeFunction(stream);
    }
    
    // IMPORTANT: check derived classes FIRST in the following if-clauses
    if(dynamic_cast<ofc::functionLib::NormalizedPolynom*>(f)){
        plhs[0] = convertPtr2Mat<ofc::functionLib::NormalizedPolynom>((ofc::functionLib::NormalizedPolynom*)f);
    }
    else if(dynamic_cast<ofc::functionLib::Polynom*>(f)){
        plhs[0] = convertPtr2Mat<ofc::functionLib::Polynom>((ofc::functionLib::Polynom*)f);
    }
    else if(dynamic_cast<ofc::functionLib::PiecewiseFunction*>(f)){
        plhs[0] = convertPtr2Mat<ofc::functionLib::PiecewiseFunction>((ofc::functionLib::PiecewiseFunction*)f);
    }
    else if(dynamic_cast<ofc::functionLib::Function*>(f)){
        plhs[0] = convertPtr2Mat<ofc::functionLib::Function>(f);
    }
    return;
    
}
