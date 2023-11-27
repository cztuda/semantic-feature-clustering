#pragma once

#include <string>
#include "mex.h"
#include "class_handle.hpp"

#include "MFun.h"
#include "SingleVariableMFun.h"
#include "../models/optimalcontrolproblem.h"
#include "../functionReimplement/PiecewiseFunction.h"
#include "../functionReimplement/SingleVariableFunction.h"

using namespace std;
using namespace ofc;
using namespace ofc::math;
using namespace ofc::models;

ofc::functionLib::SingleVariableFunction* interface_setInitialGuess(const std::string& name, const mxArray* prhs[], initialGuess::InitialGuessType& Type) {
    
    
    //MEX_GET_STRING
    std::string type;
    {
        if(!mxIsChar(prhs[1+2])){
            char tmp[256];
            snprintf(tmp, 256, "%s: Input 2 must be a string.", name.c_str());
            mexErrMsgTxt(tmp);
        }
        mwSize buflen = mxGetN(prhs[1+2])*sizeof(mxChar)+1;
        char* c_array = new char[buflen];
        mxGetString(prhs[1+2],c_array,buflen);
        type = std::string(c_array);
        delete [] c_array;
    }
    
    if(!type.compare("Lagrange")){
        Type = initialGuess::Lagrange;
    }
    else if(!type.compare("State")){
        Type = initialGuess::State;
    }
    else if(!type.compare("Control")){
        Type = initialGuess::Control;
    }
    else if(!type.compare("StateWithLagrange")){
        Type = initialGuess::StateWithLagrange;
    }
    else {
        mexErrMsgTxt("setInitialGuess: Unexpected type.");
    }
    
    // reset the function if 0 is given
    if(mxIsScalar(prhs[2]) && mxIsDouble(prhs[2]) && mxGetScalar(prhs[2]) == 0){
        return 0;
        
    }
    
    if(!mxIsStruct(prhs[2])){
        mexErrMsgTxt("Given argument must be a struct object.");
    }
    const mxArray* ufun = prhs[2];
    int state = (int) mxGetPr(mxGetField(ufun, 0, "state"))[0];
    
    ///////////// matlab-handle /////////////
    if(state == 1){
        mxArray* handle = mxDuplicateArray(mxGetField(ufun, 0, "functionHandle"));
        int nin = (int) mxGetPr(mxGetField(ufun, 0, "n_input"))[0];
        int nout = (int) mxGetPr(mxGetField(ufun, 0, "n_output"))[0];
        if(nin != 1){
            mexErrMsgTxt("setInitialGuess: Matlab handle must expect a scalar input.");
        }
        functionLib::SingleVariableFunction* fun = new SingleVariableMFun(handle, nout);
        
        handle = mxCreateDoubleMatrix(1, 1, mxREAL); // this causes a memory leak that causes MATLAB not to destroy the handle
        
        return fun;
        
    }
    
    ///////////// cpp-ND-ND-handle /////////////
    else if(state == -2){
        functionLib::Function* fun = convertMat2Ptr<functionLib::Function>(mxGetField(ufun, 0, "instanceHandle"));
        functionLib::SingleVariableFunction* svfun = dynamic_cast<functionLib::SingleVariableFunction*>(fun);
          if(svfun == 0) {
              mexErrMsgTxt("Given function is not a single variable function. Unable to add the given initial guess to the problem");
          }
          else {
            return svfun->copy();
          }        
    }
    
    ///////////// cpp-ND-Real-handle /////////////
    else if(state == -1){ 
        //uint64_t handleID = *((uint64_t *)mxGetData( mxGetField(ufun, 0, "instanceHandle") ));   
        functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(mxGetField(ufun, 0, "instanceHandle"));
        
        return fun->copy();  
        
    }
    ///////////// Unsupported state /////////////
    else {
        mexErrMsgTxt("Piecewise functions are not supported. Use a general function.");
    }
    return 0;
}





ofc::functionLib::Function* getFunctionFromUniversalFunction(const mxArray* univFPtr) {

    if(mxIsScalar(univFPtr) && mxIsDouble(univFPtr) && mxGetScalar(univFPtr) == 0){
      return 0;
    }
    if(!mxIsStruct(univFPtr)){
      mexErrMsgTxt("Given argument must be a struct object.");
      return 0;
    }
    
    const mxArray* ufun = univFPtr;
    int state = (int) mxGetPr(mxGetField(ufun, 0, "state"))[0];
    
    ///////////// matlab-handle /////////////
    if(state == 1){
      mxArray* handle = mxDuplicateArray(mxGetField(ufun, 0, "functionHandle"));
      int nin = (int) mxGetPr(mxGetField(ufun, 0, "n_input"))[0];
      int nout = (int) mxGetPr(mxGetField(ufun, 0, "n_output"))[0];
      if(nin != 1){
	mexErrMsgTxt("setInitialGuess: Matlab handle must expect a scalar input.");
      }
      functionLib::Function* fun = new SingleVariableMFun(handle, nout);
      
      handle = mxCreateDoubleMatrix(1, 1, mxREAL); // this causes a memory leak that causes MATLAB not to destroy the handle
      
      return fun;
    }
    
    ///////////// cpp-ND-ND-handle /////////////
    else if(state == -2){
      functionLib::Function* fun = convertMat2Ptr<functionLib::Function>(mxGetField(ufun, 0, "instanceHandle"));
      return fun->copy();
    }
    
    ///////////// cpp-ND-Real-handle /////////////
    else if(state == -1){ 
      //uint64_t handleID = *((uint64_t *)mxGetData( mxGetField(ufun, 0, "instanceHandle") ));   
      functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(mxGetField(ufun, 0, "instanceHandle"));
      
      return fun->copy();
      
    }
    ///////////// Unsupported state /////////////
    else {
      mexErrMsgTxt("Piecewise functions are not supported. Use a general function.");
      return 0;
    }
    
}




