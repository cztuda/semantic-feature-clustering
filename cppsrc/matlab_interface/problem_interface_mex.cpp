#include "mex.h"



// void mxGetString(const mxArray, char* cmd, unsigned long size){};
// void mexErrMsgTxt(const char* text){};
// void mexWarnMsgTxt(const char* text){};

#include "class_handle.hpp"
#include "../dlc/dlclass.hpp"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <memory>
#include <boost/archive/polymorphic_text_iarchive.hpp>
#include <boost/archive/polymorphic_text_oarchive.hpp>

#include "MFun.h"
#include "SingleVariableMFun.h"
#include "../functionReimplement/PiecewiseFunction.h"

#include "mex_directives.h"
#include "mex_univFun_handling.h"

#include "../util/exceptions.h"
#include "../models/modelloader.h"

#ifdef MUJOCOSIM_PROBLEM
#include "../mujoco_interface/mujocosimulatable.h"
#endif

using namespace std;
using namespace ofc;
using namespace ofc::models;
using namespace ofc::math;

// ofc::models::MujocoSimulatable* getMujocoSimulatable(ofc::models::OptimalControlProblem* p);

#ifdef MUJOCOSIM_PROBLEM
std::shared_ptr<visualization::CameraProperties> createCameraProperties(const std::vector<double>& camP);
#endif // MUJOCOSIM_PROBLEM  

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    
  // Get the command string
  char cmd[64];
  if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    mexErrMsgTxt("First input should be a command string less than 64 characters long.");
  
  
//////////////////////////////////////////////
/////////////// Static methods ///////////////
//////////////////////////////////////////////
  
  if(!strcmp("unserialize", cmd)) {
//       ofc::models::OptimalControlProblem* model;
//       serial2(&model);
//       plhs[0] = convertPtr2Mat<OptimalControlProblem>(model);
//       return;
      MEX_CHECK(unserialize, 2-1, 1, -1)  
      if(!mxIsChar(prhs[1])){
          mexErrMsgTxt("Second argument (no handleptr required) must be a string.");
      }
      
      double isFilename;
      MEX_GET_DOUBLE(unserialize, isFilename, 0)
      
      OptimalControlProblem* model = 0;
      if(isFilename) {
          std::string fname;
          MEX_GET_STRING(unserialize, fname, -1)
          std::ifstream stream(fname);
          model = unserializeModel(stream);
          stream.close();
      }
      else {
          std::string data;
          MEX_GET_STRING(unserialize, data, -1)
          std::istringstream stream(data);
          model = unserializeModel(stream);
      }
      if(model == 0){
          mexErrMsgTxt("Failed to unserialize given optimal feedback control problem description.");
          MEX_RETURN_EMPTY(0)
          return;
      }
      plhs[0] = convertPtr2Mat<OptimalControlProblem>(model);
      return;
  }
  
//////////////////////////////////////////////
//////// Constructor and desctructors ////////
//////////////////////////////////////////////
  
  // Call constructor
  if (!strcmp("new", cmd)) {
      // Check parameters
    if(nrhs < 2)
      mexErrMsgTxt("New: Second input should be a string."); 
    if (nlhs < 1)
      mexErrMsgTxt("New: One output expected.");
    // Return a handle to a new C++ instance
    std::string strname;
    MEX_GET_STRING(new, strname, -1)
    
    ofc::models::OptimalControlProblem* problem = createModel(strname);
    
    
    if(problem == 0) {
      mexErrMsgTxt("No valid input string.");
    }
    plhs[0] = convertPtr2Mat<OptimalControlProblem>(problem);
    MEX_RETURN_STRING(strname, 1)
    return;
  }
  
  // Check there is a second input, which should be the class instance handle
  if (nrhs < 2)
      mexErrMsgTxt("Second input should be a class instance handle.");
  
  // Delete
  if (!strcmp("delete", cmd)) {
      // Destroy the C++ object
      destroyObject<OptimalControlProblem>(prhs[1]);
      // Warn if other commands were ignored
      if (nlhs != 0 || nrhs != 2)
	  mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
      return;
  }
  
  // Get the class instance pointer from the second input
    OptimalControlProblem *problem = convertMat2Ptr<OptimalControlProblem>(prhs[1]);
  
    if(!strcmp("copy", cmd)){
        std::string strname;
        MEX_GET_STRING(new, strname, 0)
        ofc::models::OptimalControlProblem* problem2 = copyModel(strname, *problem);
        plhs[0] = convertPtr2Mat<OptimalControlProblem>(problem2);
        MEX_RETURN_STRING(strname, 1)
        return;
    }

    
  
///////////////////////////////////////////////
//////////// Various class methods ////////////
///////////////////////////////////////////////
  
  
  // getNumberOfStateVariables    
  if (!strcmp("getNumberOfStateVariables", cmd)) {
      // Check parameters
      if (nlhs < 0 || nrhs < 2)
	  mexErrMsgTxt("getNumberOfStateVariables: Unexpected arguments.");
      // Call the method
      int n = problem->getNumberOfStateVariables();
      plhs[0] = mxCreateDoubleScalar((double)n);
      return;
  }
  
  
  // getNumberOfControlVariables    
  if (!strcmp("getNumberOfControlVariables", cmd)) {
      // Check parameters
      if (nlhs < 0 || nrhs < 2)
	  mexErrMsgTxt("getNumberOfControlVariables: Unexpected arguments.");
      // Call the method
      int n = problem->getNumberOfControlVariables();
      plhs[0] = mxCreateDoubleScalar((double)n);
      return;
  }
  
  
  // getProblemName    
  if (!strcmp("getProblemName", cmd)) {
      // Check parameters
      if(nlhs > 1)
	mexErrMsgTxt("setProblemName: Too many output arguments.");
      if(nlhs == 0)
	return;
      // Call the method
      std::string s;
      problem->getProblemName(s);
      plhs[0] = mxCreateString(s.c_str());
      return;
  }
      
  
  // setProblemName    
  if (!strcmp("setProblemName", cmd)) {
      // Check parameters
      if (nlhs < 0 || nrhs < 3)
	mexErrMsgTxt("setProblemName: Additional argument required.");
      if(nlhs > 0)
	mexErrMsgTxt("setProblemName: Too many output arguments.");
      // Call the method
      char* name = mxArrayToString(prhs[2]);
      problem->setProblemName(name);
      mxFree(name);
      return;
  }
  
  
  //getInitialValues
  if (!strcmp("getInitialValues", cmd)) {
      MEX_CHECK(getInitialValues, 0, 2, -1)
      // Call the method
      TVectorX istate;
      TVectorX icontrol;
      
      problem->getStateAtInitialTime(istate);
      EigenVectorToMxArray(&plhs[0], istate, true);	
      
      if(nlhs == 2){
	problem->getControlAtInitialTime(icontrol);
	EigenVectorToMxArray(&plhs[1], icontrol, true);	  
      }
      return;
  }
  
  
  // setInitialValues    
  if (!strcmp("setInitialValues", cmd)) {
    MEX_CHECK(setInitialValues, 2, 1, -1)
    
    int sl = mxGetNumberOfElements(prhs[2]);
    int cl = mxGetNumberOfElements(prhs[3]);
    if( (!isRealNonemptyVector(prhs[2])) || (!isRealNonemptyVector(prhs[3])) )
      mexErrMsgTxt("setInitialValues: Inputs must be real, nonempty row or column vectors.");
    TVectorX state; CppArrayToEigenVector(mxGetPr(prhs[2]), sl, state);
    TVectorX control; CppArrayToEigenVector(mxGetPr(prhs[3]), cl, control);
    
    bool res = problem->setValuesAtInitialTime(state, control);
    MEX_RETURN_DOUBLE((double)res, 0)
    return;
  }
    
  // getFinalValues    
  if (!strcmp("getFinalValues", cmd)) {
    MEX_CHECK(getFinalValues, 0, 2, -1)
    
    MEX_RETURN_VECTOR_X(problem, getStateAtFinalTime, 0)
    MEX_RETURN_VECTOR_X(problem, getControlAtFinalTime, 1)
    
    return;      
  }
    
        
  // setFinalValues    
  if (!strcmp("setFinalValues", cmd)) {
    MEX_CHECK(setFinalValues, 2, 1, -1)
    
    TVectorX state;
    TVectorX control;
    MEX_GET_TVectorX(setFinalValues, state, 0)
    MEX_GET_TVectorX(setFinalValues, control, 1)
    
    bool res = problem->setValuesAtFinalTime(state, control);
    
    MEX_RETURN_DOUBLE((double)res, 0)
    return;      
  }
  
   
  MEX_FUN_GETTERSETTER_TVECTOR_X_REF(problem, StateVariableLowerBound)
  
  MEX_FUN_GETTERSETTER_TVECTOR_X_REF(problem, StateVariableUpperBound)
  
  MEX_FUN_GETTERSETTER_TVECTOR_X_REF(problem, ControlVariableLowerBound)
  
  MEX_FUN_GETTERSETTER_TVECTOR_X_REF(problem, ControlVariableUpperBound)
  
  
  MEX_FUN_GETTERSETTER_OBJVECTOR_X_REF(problem, StateIsFixedAtInitialTime, VectorXb)
  
  MEX_FUN_GETTERSETTER_OBJVECTOR_X_REF(problem, StateIsFixedAtFinalTime, VectorXb)
  
  MEX_FUN_GETTERSETTER_OBJVECTOR_X_REF(problem, ControlIsFixedAtInitialTime, VectorXb)
  
  MEX_FUN_GETTERSETTER_OBJVECTOR_X_REF(problem, ControlIsFixedAtFinalTime, VectorXb)
  
    
  MEX_FUN_GETTERSETTER_OBJVECTOR_X_REF(problem, StateIsUnconstrainedAngle, VectorXb)
  
  MEX_FUN_GETTERSETTER_OBJVECTOR_X_REF(problem, ControlIsUnconstrainedAngle, VectorXb)

  
  if(!strcmp("setInitialGuess2", cmd)) {
      MEX_CHECK(setInitialGuess, 2, 0, -1)
      
      initialGuess::InitialGuessType Type;
      functionLib::SingleVariableFunction* fun = interface_setInitialGuess("setInitialGuess", prhs, Type);
      
      if(!problem->setInitialEstimate(fun, Type)) {
          mexErrMsgTxt("Unable to add the given initial guess to the problem");
      }
      return;
  }
  
  if(!strcmp("setInitialGuess", cmd)) {
      //       MEX_CHECK(setInitialGuess, 2, 0, -1)
      //       
      //       initialGuess::InitialGuessType Type;
      //       functionLib::Function* fun = interface_setInitialGuess("setInitialGuess", prhs, Type);
      //       
      //       if(!problem->setInitialGuess(fun, Type)) {
      //           mexErrMsgTxt("Unable to add the given initial guess to the problem");
      //       }
      //       return;
      
      //ATTENTION: The code for this case exists multiple times in the files
      // - problem_interface_mex.cpp
      // - psopt_interface_SP_mex.cpp
      // - dircol_interface_mex.cpp
      MEX_CHECK(setInitialGuess, 2, 0, -1)
      
      // determine, if this initial guess is for the Lagrange term, the state or the control
      std::string type;
      MEX_GET_STRING(setInitialGuess, type, 1)
      initialGuess::InitialGuessType Type;
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
      
      // reset the function, if 0 is given
      if(mxIsScalar(prhs[2]) && mxIsDouble(prhs[2]) && mxGetScalar(prhs[2]) == 0){
          problem->setInitialEstimate(0, Type);
          return;
      }
      
      if(!mxIsStruct(prhs[2])){
          mexErrMsgTxt("Given argument must be a struct object.");
          return;
      }    
      const mxArray* ufun = prhs[2];
      int state = (int) mxGetPr(mxGetField(ufun, 0, "state"))[0];
      ///////////// matlab-handle /////////////
      if(state == 1){ 
          mxArray* handle = mxDuplicateArray(mxGetField(ufun, 0, "functionHandle"));
          int nin = (int) mxGetPr(mxGetField(ufun, 0, "n_input"))[0];
          if(nin != 1){
              mexErrMsgTxt("setInitialGuess: Given Matlab handle must expect a scalar input." );
          }
          int nout = (int) mxGetPr(mxGetField(ufun, 0, "n_output"))[0];
          functionLib::SingleVariableFunction* fun = new SingleVariableMFun(handle, nout);
          if(!problem->setInitialEstimate(fun, Type)) mexErrMsgTxt("Unable to add the given initial guess to the problem");
          
          handle = mxCreateDoubleMatrix(1, 1, mxREAL); // this causes a memory leak such that MATLAB does not destroy the handle
      }
      ///////////// cpp-ND-ND-handle /////////////
      else if(state == -2){ 
          //uint64_t handleID = *((uint64_t *)mxGetData( mxGetField(ufun, 0, "instanceHandle") ));
          functionLib::Function * fun = convertMat2Ptr<functionLib::Function>(mxGetField(ufun, 0, "instanceHandle"));
          functionLib::SingleVariableFunction* svfun = dynamic_cast<functionLib::SingleVariableFunction*>(fun);
          if(svfun == 0) {
              mexErrMsgTxt("Given function is not a single variable function. Unable to add the given initial guess to the problem");
          }
          else {
            if(!problem->setInitialEstimate(svfun->copy(), Type)) mexErrMsgTxt("Unable to add the given initial guess to the problem");
          }
          
      }
      ///////////// cpp-ND-Real-handle /////////////
      else if(state == -1){ 
          //uint64_t handleID = *((uint64_t *)mxGetData( mxGetField(ufun, 0, "instanceHandle") ));   
          functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(mxGetField(ufun, 0, "instanceHandle"));
          if(!problem->setInitialEstimate(fun->copy(), Type)) mexErrMsgTxt("Unable to add the given initial guess to the problem");
      }
      ///////////// Unsupported state /////////////
      else {
          mexErrMsgTxt("Given struct object has unsupported state.");
          return;
      }
      
      return;
  }
  
  
  if (!strcmp("setStartTime", cmd)) {
    MEX_CHECK(setStartTime, 1, 0, -1)
    double t0;
    MEX_GET_DOUBLE(setStartTime, t0, 0)
    
    
    problem->setInitialTime(t0);
    return;      
  }
  
  if (!strcmp("setEndTime", cmd)) {
    MEX_CHECK(setEndTime, 3, 0, -1)
    TVectorX vector(3);
    MEX_GET_DOUBLE(setEndTime, vector(0), 0)
    MEX_GET_DOUBLE(setEndTime, vector(1), 1)
    MEX_GET_DOUBLE(setEndTime, vector(2), 2)
    
    if(vector(1) > vector(0) || vector(2) < vector(0))
      printfFnc("problem_interface:setEndTime: Invalid boundaries.");
    
    problem->setFinalTime(vector(0), vector(1), vector(2));
    return;      
  }
  
  if (!strcmp("getEndTime", cmd)) {
    MEX_CHECK(getEndTime, 0, 1, -1)
    if(nlhs > 0) { 
      double tf, ub, lb;
      problem->getFinalTime(tf, lb, ub);
      plhs[0] = mxCreateDoubleMatrix( 3, 1, mxREAL);
      double* mxdata = mxGetPr(plhs[0]);
      mxdata[0] = tf;
      mxdata[1] = lb;
      mxdata[2] = ub;
    }
    return;      
  }
  
    if (!strcmp("getStartTime", cmd)) {
    MEX_CHECK(getStartTime, 0, 1, -1)
    if(nlhs > 0) { 
      const double t0 = problem->getInitialTime(); 
      plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL);
      double* mxdata = mxGetPr(plhs[0]);
      mxdata[0] = t0;
    }
    return;      
  }
  
  if(!strcmp("getDynamicMode", cmd)){
      MEX_CHECK(getDynamicMode, 0, 1, -1)
      double d = problem->getDynamicMode();
      MEX_RETURN_DOUBLE(d, 0)
      return;
  }
  
  if(!strcmp("setDynamicMode", cmd)){
      MEX_CHECK(setDynamicMode, 1, 0, -1)
      double var;
      MEX_GET_DOUBLE(setDynamicMode, var, 0)
      bool succ = problem->setMode(var);
      MEX_RETURN_DOUBLE(succ, 0)
      return;
  }
  
  if(!strcmp("getDynamicModeDescription", cmd)){
      MEX_CHECK("getDynamicModeDescription", 0, 1, -1)
      std::string info = problem->modeDescription();
      MEX_RETURN_STRING(info, 0)
      return;
  }
  
  
  if(!strcmp("evaluateDynamics", cmd)){
    MEX_CHECK(evaluateDynamics, 3, 1, -1) // input: state, control, time
    if(nlhs == 0) 
      return;
    double* state; 	int ls = problem->getNumberOfStateVariables();
    double* control; 	int lc = problem->getNumberOfControlVariables();
    const double* params = problem->getParameters(); int lp = problem->getNumberOfParameters();
    
    int n = 1; // the number of points that have to be evaluated
    if(mxGetNumberOfElements(prhs[0+2])%ls == 0 && mxGetNumberOfElements(prhs[0+2])/ls > 1 ){
        n = (int) mxGetNumberOfElements(prhs[2])/ls;
    }
    std::vector<double> tmp(n, 0);
    
    MEX_GET_DOUBLE_PTR(evaluateDynamics, state, ls*n, 0)
    MEX_GET_DOUBLE_PTR(evaluateDynamics, control, lc*n, 1)
    
    plhs[0] = mxCreateDoubleMatrix(ls, n, mxREAL); 
    double *f = mxGetPr(plhs[0] ); 
    
    for(size_t i = 0; i < n; ++i){
        problem->evaluate(&f[i*ls], &state[i*ls], &control[i*lc], params, ls, lc, lp);
    }
    return;
  }
  
  if(!strcmp("evaluateJacobians", cmd)){
    MEX_CHECK(evaluateJacobians, 3, 3, -1) // input: state, control, time
    if(nlhs == 0) 
      return;
    double* state; 	int ls = problem->getNumberOfStateVariables();
    double* control; 	int lu = problem->getNumberOfControlVariables();
    const double* params = problem->getParameters(); int lp = problem->getNumberOfParameters();
    int lc = problem->getNumberOfConstants();
    
    int n = 1; // the number of points that have to be evaluated
    if(mxGetNumberOfElements(prhs[0+2])%ls == 0 && mxGetNumberOfElements(prhs[0+2])/ls > 1 ){
        n = (int) mxGetNumberOfElements(prhs[2])/ls;
    }
    std::vector<double> tmp(n, 0);
    
    MEX_GET_DOUBLE_PTR(evaluateJacobians, state, ls*n, 0)
    MEX_GET_DOUBLE_PTR(evaluateJacobians, control, lu*n, 1)
    
    
    double* Jx = 0;  size_t nJx = ls*ls;
    double* Ju = 0;  size_t nJu = ls*lu;
    double* Jc = 0;  size_t nJc = ls*lc;
    double* Jp = 0;  size_t nJp = ls*lp;
    
    const int isU = (int) (nlhs > 1);
    const int isC = (int) (nlhs > 2);
    const int isP = (int) (nlhs > 3);
    
    plhs[0] = mxCreateDoubleMatrix(nJx, n, mxREAL); 
    Jx = mxGetPr( plhs[0] ); 
    if(nlhs > 1){
        plhs[1] = mxCreateDoubleMatrix(nJu, n, mxREAL); 
        Ju = mxGetPr( plhs[1] ); 
    }
    if(nlhs > 2){
        plhs[2] = mxCreateDoubleMatrix(nJc, n, mxREAL); 
        Jc = mxGetPr( plhs[2] ); 
    }
    if(nlhs > 3){
        plhs[3] = mxCreateDoubleMatrix(nJp, n, mxREAL); 
        Jp = mxGetPr( plhs[3] ); 
    }
    
    // evaluate the jacobian for each point
    for(size_t i = 0; i < n; ++i){
        problem->dynamicsJacobians(&Jx[i*nJx], &Ju[i*nJu*isU], &Jc[i*nJc*isC], &Jp[i*nJp*isP], &state[i*ls], &control[i*lc], params, ls, lu, lp);
    }
    return;
  }
  
  
  if(!strcmp("evaluateLagrangeTerm", cmd)){
    MEX_CHECK(evaluateLagrangeTerm, 3, 1, -1) // input: state, control, time
    if(nlhs == 0) 
      return;
    double* state; 	int ls = problem->getNumberOfStateVariables();
    double* control; 	int lc = problem->getNumberOfControlVariables();
    const double* params = problem->getParameters(); int lp = problem->getNumberOfParameters();
    
    int n = 1; // the number of points that have to be evaluated
    if(mxGetNumberOfElements(prhs[0+2])%ls == 0 && mxGetNumberOfElements(prhs[0+2])/ls > 1 ){
        n = (int) mxGetNumberOfElements(prhs[2])/ls;
    }
    std::vector<double> tmp(n, 0);
    
    MEX_GET_DOUBLE_PTR(evaluateLagrangeTerm, state, ls*n, 0)
    MEX_GET_DOUBLE_PTR(evaluateLagrangeTerm, control, lc*n, 1)
    
    
    plhs[0] = mxCreateDoubleMatrix(1, n, mxREAL); 
    double *f = mxGetPr(plhs[0] ); 
        
    for(size_t i = 0; i < n; ++i){
        problem->objectiveLagrangeTermIntegrand(f[i], ls, lc, lp, &state[i*ls], &control[i*lc], params);
    }
    return;
  }

  if(!strcmp("evaluateMayerTerm", cmd)){
    MEX_CHECK(evaluateMayerTerm, 5, 1, -1) // input: stateL, controlL, stateR, controlR, time
    if(nlhs == 0) 
      return;
    double* stateL; 	double* stateR; 	int ls = problem->getNumberOfStateVariables();
    double* controlL; 	double* controlR; 	int lc = problem->getNumberOfControlVariables();
    double t;
    const double* params = problem->getParameters(); int lp = problem->getNumberOfParameters();
    
    MEX_GET_DOUBLE_PTR(evaluateMayerTerm, stateL, ls, 0)
    MEX_GET_DOUBLE_PTR(evaluateMayerTerm, controlL, lc, 1)
    MEX_GET_DOUBLE_PTR(evaluateMayerTerm, stateR, ls, 2)
    MEX_GET_DOUBLE_PTR(evaluateMayerTerm, controlR, lc, 3)
    MEX_GET_DOUBLE(evaluateMayerTerm, t, 4)
    
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL); 
    double *f = mxGetPr(plhs[0] ); 
    
    problem->objectiveMayerTerm(*f, t, ls, lc, lp, stateL, controlL, params, stateR, controlR);
    return;
  }
  
  if (!strcmp("getParameters", cmd)) {
    MEX_CHECK(getParameters, 0, 1, -1)
      int len = problem->getNumberOfParameters();
    if(nlhs > 0) {
      if(len > 0){
	MEX_RETURN_DOUBLE_PTR2_COPYDATA(problem->getParameters, len, 0);
      }
      else {
	plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL); 
      }	
    }
    return;      
  }
  
  
  if(!strcmp("getParameter", cmd)) {
    MEX_CHECK(getParameter, 1, 1, -1)
    double value;
    if(mxIsDouble(prhs[2])){
        int index;
        MEX_GET_DOUBLE(getParameter, index, 0)
        value = problem->getParameter(index);
    }
    else if(mxIsChar(prhs[2])){
        std::string sIndex;
        MEX_GET_STRING(getParameter, sIndex, 0)
        value = problem->getParameter(sIndex);
    }
    else {
        mexErrMsgTxt("getParameter: Unexpected format of the given index.");
    }
    MEX_RETURN_DOUBLE(value, 0)
    return;      
  }
  
  if (!strcmp("setParameter", cmd)) {
    MEX_CHECK(setParameter, 2, 1, -1)
    double index, value;
    bool retval;
    MEX_GET_DOUBLE(setParameter, value, 1)
    if(mxIsDouble(prhs[2])){
        MEX_GET_DOUBLE(setParameter, index, 0)
        retval = problem->setParameter(index, value);
    }
    else if(mxIsChar(prhs[2])){
        std::string sIndex;
        MEX_GET_STRING(setParameter, sIndex, 0)
        retval = problem->setParameter(sIndex, value);
    }
    else {
        mexErrMsgTxt("setParameter: Unexpected format of the first input parameter.");
    }
    MEX_RETURN_DOUBLE((double) retval, 0)
    return;      
  }
    
  if (!strcmp("isParameter", cmd)) {
      MEX_CHECK(isParameter, 1, 1, -1)
      std::string sIndex;
        MEX_GET_STRING(isParameter, sIndex, 0)
        double retval = static_cast<double>(problem->isParameter(sIndex));
        MEX_RETURN_DOUBLE(retval, 0)
        return;
  }
  
  if(!strcmp("getParameterDescription", cmd)) {
    MEX_CHECK("getParameterDescription", 0, 1, -1)
    std::string info = "";
    if(nrhs > 2){
        double index;
        MEX_GET_DOUBLE(getParameterDescription, index, 0);
        info = problem->getParameterDescription(static_cast<size_t>(index));
    }
    else{
        std::string info = problem->getParameterDescriptions();
    }
    MEX_RETURN_STRING(info, 0)
//     const char* info = problem->getParameterDescription().c_str();
//     int ilen = strlen(info)+1;
//     if(nlhs > 0){
//       char* output_buf = (char*)mxCalloc(ilen, sizeof(char));
//       memcpy(output_buf, info, ilen* sizeof(char));
//       plhs[0] = mxCreateString(output_buf);
//     }
    return;

  }
  
    if (!strcmp("getConstants", cmd)) {
    MEX_CHECK(getConstants, 0, 1, -1)
      int len = problem->getNumberOfConstants();
    if(nlhs > 0) {
      if(len > 0){
	MEX_RETURN_DOUBLE_PTR2_COPYDATA(problem->getConstants, len, 0);
      }
      else {
	plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL); 
      }	
    }
    return;      
  }
  
  if(!strcmp("getConstant", cmd)) {
    MEX_CHECK(getConstant, 1, 1, -1)
    double value;
    if(mxIsDouble(prhs[2])){
        int index;
        MEX_GET_DOUBLE(getConstant, index, 0)
        value = problem->getConstant(index);
    }
    else if(mxIsChar(prhs[2])){
        std::string sIndex;
        MEX_GET_STRING(getConstant, sIndex, 0)
        value = problem->getConstant(sIndex);
    }
    else {
        mexErrMsgTxt("getConstant: Unexpected format of the given index.");
    }
    MEX_RETURN_DOUBLE(value, 0)
    return;      
  }
  
  if (!strcmp("setConstant", cmd)) {
    MEX_CHECK(setConstant, 2, 1, -1)
    double index, value;
    bool retval;
    MEX_GET_DOUBLE(setConstant, value, 1)
    if(mxIsDouble(prhs[2])){
        MEX_GET_DOUBLE(setConstant, index, 0)
        retval = problem->setConstant(index, value);
    }
    else if(mxIsChar(prhs[2])){
        std::string sIndex;
        MEX_GET_STRING(setConstant, sIndex, 0)
        retval = problem->setConstant(sIndex, value);
    }
    else {
        mexErrMsgTxt("setConstant: Unexpected format of the first input parameter.");
    }
    MEX_RETURN_DOUBLE((double) retval, 0)
    return;      
  }
  
  if (!strcmp("isConstant", cmd)) {
      MEX_CHECK(isConstant, 1, 1, -1)
      std::string sIndex;
        MEX_GET_STRING(isConstant, sIndex, 0)
        double retval = static_cast<double>(problem->isConstant(sIndex));
        MEX_RETURN_DOUBLE(retval, 0)
        return;
  }
  
  if(!strcmp("getConstantDescription", cmd)) {
    MEX_CHECK(getConstantDescription, 0, 1, -1)
    std::string info = "";
    if(nrhs > 2){
        double index;
        MEX_GET_DOUBLE(getConstantDescription, index, 0);
        info = problem->getConstantDescription(static_cast<size_t>(index));
    }
    else{
        info = problem->getConstantDescriptions();
    }
    MEX_RETURN_STRING(info, 0)
    return;

  }

  if(!strcmp("serialize", cmd)) {
//       serial1(problem);
//       return;
    MEX_CHECK("serialize", 0, 1, -1)
    std::ostringstream stream;
    serializeModel(stream, problem);
    MEX_RETURN_STRING(stream.str(), 0)
    return;
  }
  
  
  
//////////////////////////////////////////////
//////// Mujoco specific functin calls ///////
//////////////////////////////////////////////
#ifdef MUJOCOSIM_PROBLEM

OptimalControlProblem *p = convertMat2Ptr<OptimalControlProblem>(prhs[1]);
MujocoSimulatable* sim = nullptr;
try {
    sim = dynamic_cast<ofc::models::MujocoSimulatable*>(p);            
}
catch(int e){
    throw ofc::exception::InconsistentInputError("Given problem description is not a mujoco simulatable instance.");
}


if(!strcmp("mujoco_simulateStep", cmd)) {
    MEX_CHECK("mujoco_simulateStep", 1, 2, -1)
    
    int nq, nv, na, nu, nB;
    sim->getMujocoDimensions(nq, nv, na, nu, nB);
    const int ls = nq+nv+na;
    int lds = 2*nv+na;
    const int lc = nu; //nu+nv+nB;
    
    int n = 1; // the number of points that have to be evaluated
    bool stateGiven = true;
    if(mxGetNumberOfElements(prhs[0+2])%ls == 0 && mxGetNumberOfElements(prhs[0+2])/ls > 1 ){
        n = (int) mxGetNumberOfElements(prhs[2])/ls;
    }
    else if(mxGetNumberOfElements(prhs[0+2])%lc == 0 && mxGetNumberOfElements(prhs[0+2])/lc > 1){
        n = (int) mxGetNumberOfElements(prhs[2])/lc;
        stateGiven = false;
    }
    else if(mxGetNumberOfElements(prhs[0+2]) != ls && mxGetNumberOfElements(prhs[0+2]) != lc){
        MEX_ERROR("mujoco_simulateStep", "%s: Unexpected size of first input argument.") 
    }
    else {
        stateGiven = mxGetNumberOfElements(prhs[0+2]) == ls;
    }
    Eigen::MatrixXd x_new(ls, n);
    double* ctrl = nullptr;
    double* x = nullptr;
    double* t = nullptr;
    bool getTimeDerivative = false;
    switch(nrhs-2){
        case 1:
            MEX_GET_DOUBLE_PTR(mujoco_simulateStep, ctrl, lc*n, 0)
            break;
        case 2:
            if(stateGiven){
                MEX_GET_DOUBLE_PTR(mujoco_simulateStep, x, ls*n, 0)
                MEX_GET_DOUBLE_PTR(mujoco_simulateStep, ctrl, lc*n, 1)
            }
            else if(mxIsLogical(prhs[0+2+1])) {
                MEX_GET_DOUBLE_PTR(mujoco_simulateStep, ctrl, lc*n, 0)
                getTimeDerivative = *mxGetLogicals(prhs[0+2+1]);
            }
            else {
                MEX_GET_DOUBLE_PTR(mujoco_simulateStep, ctrl, lc*n, 0)
                MEX_GET_DOUBLE_PTR(mujoco_simulateStep, t, n, 1)
            }
            break;            
        case 3:
            MEX_GET_DOUBLE_PTR(mujoco_simulateStep, x, ls*n, 0)
            MEX_GET_DOUBLE_PTR(mujoco_simulateStep, ctrl, lc*n, 1)
            if(mxIsLogical(prhs[0+2+1])) {
                getTimeDerivative = *mxGetLogicals(prhs[0+2+2]);
            }
            else {
                MEX_GET_DOUBLE_PTR(mujoco_simulateStep, t, n, 2)
            }
            break;
        case 4:
            MEX_GET_DOUBLE_PTR(mujoco_simulateStep, x, ls*n, 0)
            MEX_GET_DOUBLE_PTR(mujoco_simulateStep, ctrl, lc*n, 1)
            MEX_GET_DOUBLE_PTR(mujoco_simulateStep, t, n, 2)
            getTimeDerivative = *mxGetLogicals(prhs[0+2+3]);
            break;
        default:
            MEX_ERROR("mujoco_simulateStep", "%s: Unexpected number of input arguments.") 
    }
    if(getTimeDerivative){
        lds = ls;
    }
    Eigen::MatrixXd dx(lds, n);
    Eigen::Map<Eigen::MatrixXd> control(ctrl, lc, n);
    Eigen::Map<Eigen::MatrixXd> state(x, 0, 0);
    
    double time = 0;
    if(t != nullptr){
        time = *t;
    }
    else {
        time = sim->getSimulationCurrentTime();
    }
    if(stateGiven){
        new (&state) Eigen::Map<Eigen::MatrixXd>(x, ls, n);
    }
    
    for(size_t i = 0; i < n; ++i){
        if(stateGiven){
            sim->simulateStep(dx, state, control, time, getTimeDerivative);
        }
        else{
            sim->simulateStep(dx, control, time, getTimeDerivative);
        }
        sim->getSimulationCurrentState(x_new.col(i).data(), ls);
    }
    
    plhs[0] = mxCreateDoubleMatrix(ls, n, mxREAL); 
    double *f2 = mxGetPr(plhs[0]); 
    memcpy(f2, x_new.data(), sizeof(double)*ls*n);
    
    if(nlhs > 1){
        plhs[1] = mxCreateDoubleMatrix(lds, n, mxREAL); 
        double *f = mxGetPr(plhs[1]);
        memcpy(f, dx.data(), sizeof(double)*lds*n);
    }
    
    return;
}

if(!strcmp("mujoco_forward", cmd)) {
    MEX_CHECK("mujoco_forward", 3, 2, -1)
    
    if(nlhs == 0) {
        return;
    }
    int nq, nv, na, nu, nB;
    sim->getMujocoDimensions(nq, nv, na, nu, nB);
    int ls = nq+nv+na;
    int lc = nu; //nu+nv+nB;
    
    if(mxGetNumberOfElements(prhs[0+2]) != ls){
        throw ofc::exception::Error("Unexpected size of state vector.");
    }
    if(mxGetNumberOfElements(prhs[1+2]) != lc){
        throw ofc::exception::Error("Unexpected size of control vector.");
    }
    Eigen::MatrixXd dx(ls, 1);
    double time;
    double* x_data = nullptr;
    double* u_data = nullptr;
    MEX_GET_DOUBLE_PTR(mujoco_forward, x_data, ls, 0)
    MEX_GET_DOUBLE_PTR(mujoco_forward, u_data, lc, 1)
    MEX_GET_DOUBLE(mujoco_forward, time, 2)
    
    sim->forward(dx, Eigen::Map<Eigen::MatrixXd>(x_data, ls, 1), Eigen::Map<Eigen::MatrixXd>(u_data, lc, 1), time);
    
    if(nlhs > 0){
        plhs[0] = mxCreateDoubleMatrix(ls, 1, mxREAL); 
        double *f = mxGetPr(plhs[0]); 
        memcpy(f, dx.data(), sizeof(double)*ls);
    }
    
    return;
}

if(!strcmp("mujoco_getOptionData", cmd)) {
    MEX_CHECK("mujoco_getOptionData", 0, 1, -1)
    
    std::vector<double> opts;
    sim->getSimulationOptions(opts);
    
    if(nlhs > 0) {
        plhs[0] = mxCreateDoubleMatrix(opts.size(), 1, mxREAL);
        double* mxdata = mxGetPr(plhs[0]);
        memcpy(mxdata, opts.data(), opts.size()*sizeof(double));
    }
    return;
}

if(!strcmp("mujoco_setOptionData", cmd)) {
    MEX_CHECK("mujoco_setOptionData", 1, 0, -1)
    
//     int n = mxGetNumberOfElements(prhs[0+2]);
    std::vector<double> opt;
//     std::vector<double> opt(n);
//     double* raw = nullptr;
//     MEX_GET_DOUBLE_PTR(mujoco_setOptionData, raw, n, 0)
//     memcpy(opt.data(), raw, n*sizeof(double));
    MEX_GET_STD_VECTOR_DOUBLE(mujoco_setOptionData, opt, 0)
    sim->setSimulationOptions(opt);
    return;
}

if(!strcmp("mujoco_getCurrentState", cmd)) {
    MEX_CHECK("mujoco_getCurrentState", 0, 1, -1)
    if(nlhs == 0) {
        return;
    }
    int nq, nv, na, nu, nB;
    sim->getMujocoDimensions(nq, nv, na, nu, nB);
    int ls = nq+nv+na;
    
    plhs[0] = mxCreateDoubleMatrix(ls, 1, mxREAL); 
    double *f = mxGetPr(plhs[0] ); 
    
    sim->getSimulationCurrentState(f, ls);
    return;
}

if(!strcmp("mujoco_setCurrentState", cmd)) {
    MEX_CHECK("mujoco_setCurrentState", 1, 1, -1)
    int nq, nv, na, nu, nB;
    sim->getMujocoDimensions(nq, nv, na, nu, nB);
    int ls = nq+nv+na;
    
    double* x;
    MEX_GET_DOUBLE_PTR(mujoco_setCurrentState, x, ls, 0)
    
    sim->setSimulationCurrentState(x, ls);
    return;
}

//     if(!strcmp("mujoco_getCurrentVelocity", cmd)) {
//         MEX_CHECK("mujoco_getCurrentVelocity", 0, 1, -1)
//         if(nlhs == 0) {
//             return;
//         }
//         int nq, nv, na, nu, nB;
//         sim->getMujocoDimensions(nq, nv, na, nu, nB);
//         int ls = 2*nv+na;
//         
//         plhs[0] = mxCreateDoubleMatrix(ls, 1, mxREAL); 
//         double *f = mxGetPr(plhs[0] ); 
//         
//         sim->getSimulationCurrentVelocity(f, ls);
//         return;
//     }

if(!strcmp("mujoco_setCurrentTime", cmd)) {
    MEX_CHECK("mujoco_setCurrentTime", 1, 1, -1)
    
    double time;
    MEX_GET_DOUBLE(mujoco_setCurrentTime, time, 0)
    
    
    sim->setSimulationCurrentTime(time);
    return;
}

if(!strcmp("mujoco_getCurrentTime", cmd)) {
    MEX_CHECK("mujoco_getCurrentTime", 0, 1, -1)
    if(nlhs == 0) {
        return;
    }
    
    double time = sim->getSimulationCurrentTime();
    
    MEX_RETURN_DOUBLE(time, 0)        
    return;
}

if(!strcmp("mujoco_getDimensions", cmd)) {
    MEX_CHECK("mujoco_getDimensions", 0, 5, -1)
    if(nlhs == 0) {
        return;
    }
    int nq, nv, na, nu, nB;
    sim->getMujocoDimensions(nq, nv, na, nu, nB);
    
    MEX_RETURN_DOUBLE(nq, 0)
    MEX_RETURN_DOUBLE(nv, 1)
    MEX_RETURN_DOUBLE(na, 2)
    MEX_RETURN_DOUBLE(nu, 3)
    MEX_RETURN_DOUBLE(nB, 4)
    return;
}

if(!strcmp("mujoco_getDescription", cmd)) {
    MEX_CHECK("mujoco_getDescription", 0, 1, -1)
    if(nlhs == 0) {
        return;
    }
    std::string descr = sim->getMujocoDescription();
    MEX_RETURN_STRING(descr, 0);
    return;
}

if(!strcmp("mujoco_screenshot", cmd)) {
    MEX_CHECK("mujoco_screenshot", 1, 0, -1)
    std::string filename;
    MEX_GET_STRING(mujoco_screenshot, filename, 0)
    std::vector<double> camP;
    std::shared_ptr<visualization::CameraProperties> camProps(nullptr);
    if(nrhs > 3){            
        MEX_GET_STD_VECTOR_DOUBLE(mujoco_screenshot, camP, 1)
        camProps = createCameraProperties(camP);
    }
    sim->printScreenshot(filename, camProps.get());
    return;
}

if(!strcmp("mujoco_showsim", cmd)) {
    MEX_CHECK("mujoco_showsim", 1, 0, -1)
    
    functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(mxGetField(prhs[2], 0, "instanceHandle"));
    if(fun == nullptr){
        throw ofc::exception::InconsistentInputError("Given function must not be null.");
    }    
    std::shared_ptr<const functionLib::SingleVariableFunction> vfuncopy(fun->copy());
    
    std::vector<double> camP;
    std::shared_ptr<visualization::CameraProperties> camProps(nullptr);
    if(nrhs > 3){            
        MEX_GET_STD_VECTOR_DOUBLE(mujoco_showsim, camP, 1)
        camProps = createCameraProperties(camP);
    }
    sim->showSimulation(vfuncopy, camProps.get());
    return;
}

if(!strcmp("mujoco_recordsim", cmd)) {
    MEX_CHECK("mujoco_recordsim", 1, 0, -1)
    
    std::string filename;
    MEX_GET_STRING(mujoco_recordsim, filename, 0)
    
    functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(mxGetField(prhs[3], 0, "instanceHandle"));
    if(fun == nullptr){
        throw ofc::exception::InconsistentInputError("Given function must not be null.");
    }    
    std::shared_ptr<const functionLib::SingleVariableFunction> vfuncopy(fun->copy());
    
    std::vector<double> camP;
    std::shared_ptr<visualization::CameraProperties> camProps(nullptr);
    if(nrhs > 4){            
        MEX_GET_STD_VECTOR_DOUBLE(mujoco_recordsim, camP, 2)
        camProps = createCameraProperties(camP);
    }
    sim->recordSimulation(filename, vfuncopy, camProps.get());
    return;
}

if(!strcmp("mujoco_recordtraj", cmd)) {
    MEX_CHECK("mujoco_recordtraj", 1, 0, -1)
    
    std::string filename;
    MEX_GET_STRING(mujoco_recordtraj, filename, 0)
    
    functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(mxGetField(prhs[3], 0, "instanceHandle"));
    if(fun == nullptr){
        throw ofc::exception::InconsistentInputError("Given function must not be null.");
    }    
    std::shared_ptr<const functionLib::SingleVariableFunction> vfuncopy(fun->copy());
    
    std::vector<double> camP;
    std::shared_ptr<visualization::CameraProperties> camProps(nullptr);
    if(nrhs > 4){            
        MEX_GET_STD_VECTOR_DOUBLE(mujoco_recordtraj, camP, 2)
        camProps = createCameraProperties(camP);
    }
    sim->recordTrajectory(filename, vfuncopy, camProps.get());
    return;
}

if(!strcmp("mujoco_defaultCamprops", cmd)){
    MEX_CHECK("mujoco_defaultCamprops", 0, 1, -1)
    if(nlhs > 0){
        visualization::CameraProperties props;
        sim->getDefaultCamProps(props);
        std::vector<double> V;
        V.push_back(props.azimuth);
        V.push_back(props.distance);
        V.push_back(props.elevation);
        V.push_back(props.lookat[0]);
        V.push_back(props.lookat[1]);
        V.push_back(props.lookat[2]);
        MEX_RETURN_DOUBLE_PTR_COPYFROM(plhs, nlhs, 0, V);
    }
    return;
}
    
#endif // MUJOCOSIM_PROBLEM    
  
  // Got here, so command not recognized
  mexErrMsgTxt("Command not recognized.");
}


#ifdef MUJOCOSIM_PROBLEM
std::shared_ptr<visualization::CameraProperties> createCameraProperties(const std::vector<double>& camP){
    if(camP.size() == 6){
        auto camProps = shared_ptr<visualization::CameraProperties>(new visualization::CameraProperties);
        camProps->azimuth = camP[0];
        camProps->distance = camP[1];
        camProps->elevation = camP[2];
        camProps->lookat[0] = camP[3];
        camProps->lookat[1] = camP[4];
        camProps->lookat[2] = camP[5];
        
        return camProps;
    }
    return nullptr;
}
#endif // MUJOCOSIM_PROBLEM

