#include "mex.h"

#include "class_handle.hpp"
#include "mex_directives.h"


#include "../functionReimplement/TrajectoryReader.h"
#include "../functionReimplement/NormalizedPolynom.h"
#include "../functionReimplement/PiecewiseFunction.h"

#include <exception>
#include <iostream>
#include <fstream>
#include <memory>
#include <iomanip>

using namespace std;
using namespace ofc;

void sortLinearData(std::vector<double> &data, std::vector<double> *times, std::vector<double> *coefficients, int n);
void sortCubicData(std::vector<double> &data, std::vector<double> *times, std::vector<double> *coefficients, int n);
void parseAndStoreTrajectory(std::istream &stream, mxArray** mxpointer, int index);
functionLib::Function* getFunctionPointer(const mxArray* mxptr);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    // Get the command string
    char cmd[64];
    if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
      mexErrMsgTxt("First input should be a command string less than 64 characters long.");
        
    
//////////////////////////////////////////////
//////// Constructor and desctructors ////////
//////////////////////////////////////////////
  
    // -> This object is not constructed here
    
    // Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
      mexErrMsgTxt("Second input should be a class instance handle.");
    
    // Delete
    if (!strcmp("delete_piecewise", cmd)) {
        // Destroy the C++ object
        destroyObject<functionLib::PiecewiseFunction>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    if (!strcmp("delete_general", cmd)) {
        // Destroy the C++ object
        destroyObject<functionLib::Function>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    

///////////////////////////////////////////////
/////////////// Static methods ////////////////
///////////////////////////////////////////////


    if (!strcmp("generateLinear", cmd)) {
      MEX_CHECK(generateLinear, 2-1, 1, -1)
//      std::cout << "generateLinear" << std::endl;
      double* in = mxGetPr(prhs[1]);
      double* out = mxGetPr(prhs[2]);
      int len = mxGetM(prhs[2]);
      int dim = mxGetN(prhs[2]);
      if(mxGetM(prhs[1]) != len){
	mexErrMsgTxt("Number of rows of given data matrices must be the same.");
      }
      
      std::stringstream s;
      // set dimension and number of parameter=0 and number of phases=1 and number of knots and start/end time
      s << "    " << dim << "    0\n    1\n  " << len << "\n  " << setprecision(16) << in[0] << "    " << setprecision(16) << in[len-1] << "\n";
      // set dummy names
      for(int i = 0; i < dim; i++)
	s << "in" << i << "\n";
      for(int i = 0; i < len; i++){
	s << "  " << setprecision(16) << in[i];
	for(int j = 0; j < dim; j++)
	  s << "    " << setprecision(16) << out[i+len*j];
	s << "\n";
      }
      
      functionLib::PiecewiseFunction* traj;
      functionLib::TrajectoryReader reader;
      std::istringstream is(s.str());// = std::istream(s);
      reader.read(is, &traj);
      
      plhs[0] = convertPtr2Mat<functionLib::PiecewiseFunction>(traj);
      
//       nspace::VectorND res;
//       bool ress = traj->evaluate(res, 2.2);
//       printFnc("succ=%i, res=%f", (int) ress, res.data[0]);
      
      return;
    }
    
    
    /*
     * Input values:
     * (command)
     * data
     * dim
     */
    if(!strcmp("generateCubicFromData", cmd)) {
      MEX_CHECK(createCubicCppFunction, 2-1, 1, -1)
      
      double* data = mxGetPr(prhs[1]);
      int dim = (int)mxGetScalar(prhs[2]);
      int len = mxGetM(prhs[1]);
      if( (len-dim-1) % (2+4*dim) != 0){
	mexErrMsgTxt("Unexpected size of input data.");
	return;
      }
      int n = (len-dim-1) / (2+4*dim);
      int l = (len-(dim+1)) / (dim*4+2) + 1;
      
      std::stringstream s;
      // set dimension and number of switching points and number of phases and number of knots and start/end time
      s << "    " << dim << " F    0\n    1\n  " << l << "\n  " << setprecision(16) << data[0] << "    " << setprecision(16) << data[len-dim-1] << "\n";
      // set dummy names:
      for(int i = 0; i < dim; i++)
	s << "in" << i << "\n";
      int index = 0; // the position in the data mxArray
      // write the main data in correct format:
      for(int i = 0; i < n; i++){
	s << "  " << setprecision(16) << data[index] << "     " << setprecision(16) << data[index+1] << "\n";
	index +=2;
	for(int j = 0; j < dim; j++){
	  s << " " << setprecision(16) << data[index] << "    " << setprecision(16) << data[index+1] << "    " << setprecision(16) << data[index+2] << "    " << setprecision(16) << data[index+3] << "\n";
	  index += 4;
	}
      }
      // last entry:
      for(int j = 0; j < dim+1; j++)
	  s << " " << setprecision(16) << data[index++] << "\n";
      
      functionLib::PiecewiseFunction* traj;
      functionLib::TrajectoryReader reader;
      std::istringstream is(s.str());// = std::istream(s);
      bool result = reader.read(is, &traj);
    //  printfFnc(is.str().c_str())
      if(result) {
        plhs[0] = convertPtr2Mat<functionLib::PiecewiseFunction>(traj);
      }
      else {
          mexErrMsgTxt("An error occurred when parsing the given function data.");
      }
      return;
    }
    
    if(!strcmp("readFunctionFromFile", cmd)) {
      MEX_CHECK(readFunctionFromFile, 1-1, 1, -1)
      char filename[64];
      mxGetString(prhs[1], filename, sizeof(filename));
      std::ifstream stream(filename);
      if(!stream) {
		plhs[0] = mxCreateDoubleScalar(mxGetNaN());
		return;
      }
      
      functionLib::PiecewiseFunction* traj;
      functionLib::TrajectoryReader reader;
      bool succ = reader.read(stream, &traj);
      if(succ){
		plhs[0] = convertPtr2Mat<functionLib::PiecewiseFunction>(traj);
		return;	
      } else {
		mexErrMsgIdAndTxt("TrajectoryParser", "Unable to parse trajectory information.\n");
		return;
      }
      
    }
    
    /*
     * Return values:
     * n, 
     * isadj, 
     * type, 
     * nswitch, 
     * nGridpoints, 
     * t0, 
     * tf, 
     * names, 
     * times, 
     * coefficients, 
     * parameter
     */
    if (!strcmp("readDataFromFile", cmd)) {
      MEX_CHECK(readDataFromFile, 1-1, 11, -1)
      
      char filename[64];
      mxGetString(prhs[1], filename, 64);
      std::ifstream stream(filename);
      if(!stream) {
	MEX_RETURN_DOUBLE(-1, 0)
	MEX_RETURN_DOUBLE(-1, 1)
	MEX_RETURN_DOUBLE(-1, 2)
	MEX_RETURN_DOUBLE(-1, 3)
	MEX_RETURN_DOUBLE(-1, 4)
	MEX_RETURN_DOUBLE(-1, 5)
	MEX_RETURN_DOUBLE(-1, 6)
	MEX_RETURN_DOUBLE(-1, 7)
	MEX_RETURN_DOUBLE(-1, 8)
	MEX_RETURN_DOUBLE(-1, 9)
	MEX_RETURN_DOUBLE(-1, 10)
	return;
      }
      
      functionLib::PiecewiseFunction* traj;
      functionLib::TrajectoryReader reader;
      functionLib::PiecewiseFunctionData datastruct;
      bool succ = reader.read(stream, &traj, &datastruct);
      delete traj;
      
      if(succ){
	MEX_RETURN_DOUBLE(datastruct.n, 0)
	MEX_RETURN_DOUBLE(datastruct.estadj, 1)
	MEX_RETURN_DOUBLE(datastruct.np, 3)
	MEX_RETURN_VECTOR(datastruct.ngridpts, 4)
	MEX_RETURN_DOUBLE(datastruct.t0, 5)
	MEX_RETURN_DOUBLE(datastruct.tf, 6)
	
	std::string allnames = "";
	for(auto it = datastruct.names.begin(); it != datastruct.names.end(); it++)
	  allnames.append(*it).append("|");
	MEX_RETURN_STRING(allnames, 7)
	MEX_RETURN_VECTOR(datastruct.parameter, 10);
	
	int order=3;
	int knots = 0;
	for(auto it = datastruct.ngridpts.begin(); it != datastruct.ngridpts.end(); it++)
	  knots += *it;
	int cubic = (datastruct.n*(order+1)+2)*(knots-1)+datastruct.n+1;
	int linear = (datastruct.n+1)*knots;
	
	if(datastruct.doubles.size() == linear){
	  MEX_RETURN_DOUBLE(1.0, 2)
	}
	else if(datastruct.doubles.size() == cubic){
	  MEX_RETURN_DOUBLE(3.0, 2)
	}
	else{
	  MEX_RETURN_DOUBLE(-1.0, 2)
	}
	std::vector<double> times;
	std::vector<double> coefficients;
	if(datastruct.doubles.size() == cubic){
	  sortCubicData(datastruct.doubles, &times, &coefficients, datastruct.n);
	}
	else if(datastruct.doubles.size() == linear){
	  sortLinearData(datastruct.doubles, &times, &coefficients, datastruct.n);
	} else {
	  MEX_RETURN_DOUBLE(-1, 8)
	  MEX_RETURN_DOUBLE(-1, 9)
	  return;
	}
	MEX_RETURN_VECTOR(times, 8)
	MEX_RETURN_VECTOR(coefficients, 9)
      } else {
	MEX_RETURN_DOUBLE(-1, 0)
	MEX_RETURN_DOUBLE(-1, 1)
	MEX_RETURN_DOUBLE(-1, 2)
	MEX_RETURN_DOUBLE(-1, 3)
	MEX_RETURN_DOUBLE(-1, 4)
	MEX_RETURN_DOUBLE(-1, 5)
	MEX_RETURN_DOUBLE(-1, 6)
	MEX_RETURN_DOUBLE(-1, 7)
	MEX_RETURN_DOUBLE(-1, 8)
	MEX_RETURN_DOUBLE(-1, 9)
	MEX_RETURN_DOUBLE(-1, 10)
      }
      return;
      
    }
  
  if(!strcmp("unserialize", cmd)) {
      MEX_CHECK(unserialize, 2-1, 1, -1)      
      if(!mxIsChar(prhs[1])){
          mexErrMsgTxt("Second argument (no handleptr required) must be a string.");
      }
      
      double isFilename;
      MEX_GET_DOUBLE(unserialize, isFilename, 0)
      functionLib::Function* f = 0;
      if(isFilename) {
          std::string fname;
          MEX_GET_STRING(unserialize, fname, -1)
          std::ifstream stream(fname);
          functionLib::Function* f = 0;
          // 	f = functionLib::serialization::unserializeFunction(stream);
          functionLib::Function::unserialize(&f, stream);
          stream.close();
      }
      else {
          std::string data;
          MEX_GET_STRING(unserialize, data, -1)
          std::istringstream stream(data);
          f = functionLib::Function::unserialize(data);
          // 	f = functionLib::serialization::unserializeFunction(stream);	
      }
      // IMPORTANT: check derived classes FIRST in the following if-clauses
      if(dynamic_cast<functionLib::NormalizedPolynom*>(f)){
          plhs[0] = convertPtr2Mat<functionLib::NormalizedPolynom>((functionLib::NormalizedPolynom*)f);
      }
      else if(dynamic_cast<functionLib::Polynom*>(f)){
          plhs[0] = convertPtr2Mat<functionLib::Polynom>((functionLib::Polynom*)f);
      }
      else if(dynamic_cast<functionLib::PiecewiseFunction*>(f)){
          plhs[0] = convertPtr2Mat<functionLib::PiecewiseFunction>((functionLib::PiecewiseFunction*)f);
      }
      else if(dynamic_cast<functionLib::Function*>(f)){
          plhs[0] = convertPtr2Mat<functionLib::Function>(f);
      }
      return;
  }

///////////////////////////////////////////////
//////////// Various class methods ////////////
///////////////////////////////////////////////
    
    if (!strcmp("copy_piecewise", cmd)) {
      MEX_CHECK(copy_piecewise, 0, 1, -1)
      
      functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(prhs[1]);
      
      functionLib::PiecewiseFunction* traj = new functionLib::PiecewiseFunction(*fun);
      
      plhs[0] = convertPtr2Mat<functionLib::PiecewiseFunction>(traj);
      
      return;
    }
    
    
    if (!strcmp("eval_piecewise", cmd)) {
      MEX_CHECK(eval_piecewise, 1, 1, -1)
      functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(prhs[1]);

      
      size_t len = mxGetNumberOfElements(prhs[2]);
      
      plhs[0] = mxCreateDoubleMatrix(fun->NY(), len, mxREAL); 
      double *mxdata = mxGetPr(plhs[0] ); 
      
      if(nlhs == 0) return;
      
      if(len == 1){
	
	double in;
	MEX_GET_DOUBLE(eval_piecewise, in, 0)      
	fun->evaluate(mxdata, &in); // evaluate(TY & result, const TX & x){
	return;
	
      }
      else {
	
	double* in;
	MEX_GET_DOUBLE_PTR(eval_piecewise, in, static_cast<int>(len), 0);
	
	fun->evaluate(mxdata, in);
	int datasizeY = fun->NY();
	int datasizeX = fun->NX();
	mxdata += datasizeY;
	for(int i = datasizeX; i < len; i+=datasizeX) {
	  fun->evaluate(mxdata, &in[i]);
	  mxdata += datasizeY;
	}
	return;
	
      }
    }
    
    if (!strcmp("eval_derivative_piecewise", cmd)) {
      MEX_CHECK(eval_derivative_piecewise, 1, 1, -1)
      functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(prhs[1]);

      
      size_t len = mxGetNumberOfElements(prhs[2]);
      int degree;
      if(nrhs > 3) {
	MEX_GET_DOUBLE(eval_derivative_piecewise, degree, 1)
      }
      else {
	degree = 1;
      }
      
      plhs[0] = mxCreateDoubleMatrix(fun->NY(), len, mxREAL); 
      double *mxdata = mxGetPr(plhs[0] ); 
      
      if(nlhs == 0) return;
      
      if(len == 1){
	
	double in;
	MEX_GET_DOUBLE(eval_derivative_piecewise, in, 0)      
	bool result = fun->evaluateDerivative(mxdata, &in, 1, degree);
	if(!result){
	  mexErrMsgTxt("universalFunction:eval_derivative_piecewise: Evaluation of the derivative was not successful.\n");
	}
	return;
	
      }
      else {
	
	double* in;
	MEX_GET_DOUBLE_PTR(eval_derivative_piecewise, in, static_cast<int>(len), 0);
	
	fun->evaluateDerivative(mxdata, in, len, degree);
// 	int datasizeY = fun->NY();
// 	int datasizeX = fun->NX();
// 	mxdata += datasizeY;
// 	for(int i = datasizeX; i < len; i+=datasizeX) {
// 	  fun->evaluateDerivative(mxdata, &in[i], degree);
// 	  mxdata += datasizeY;
// 	}
	return;
	
      }
    }
    
    
    if(!strcmp("get_piecewise_interval", cmd)) {
      MEX_CHECK(get_piecewise_interval, 0, 2, -1)
      try {
	functionLib::PiecewiseFunction* fun = convertMat2Ptr<functionLib::PiecewiseFunction>(prhs[1]);
	functionLib::Interval ival = fun->getDefinitionInterval();
	
	MEX_RETURN_DOUBLE(ival.a, 0);
	MEX_RETURN_DOUBLE(ival.b, 1);
      }
      catch(int exception){
	mexErrMsgTxt("universalFunction:get_piecewise_interval: Unexpected error occured.\n");
	MEX_RETURN_EMPTY(0);
	MEX_RETURN_EMPTY(1);
      }
      return;
    }
    
    
    if (!strcmp("copy_general", cmd)) {
      MEX_CHECK(copy_general, 0, 1, -1)      
      functionLib::Function* fun = convertMat2Ptr<functionLib::Function>(prhs[1]);      
      functionLib::Function* traj = fun->copy();      
      plhs[0] = convertPtr2Mat<functionLib::Function>(traj);      
      return;
    }
    
    
    if (!strcmp("eval_general", cmd)) {
      MEX_CHECK(eval_general, 1, 1, -1)
//      std::cout << "eval general" << std::endl;
      functionLib::Function* fun = convertMat2Ptr<functionLib::Function>(prhs[1]);
      if(nlhs == 0) return;
      double* in;
      double* out;
      int dimsize = mxGetNumberOfDimensions(prhs[2]);
      const mwSize* dims = mxGetDimensions(prhs[2]);
      if(dimsize == 1) {
	if(fun->NX() != dims[0]){
	  mexErrMsgTxt("universalFunction:eval_general. Unexpected input dimension.");
	  return;
	}
	MEX_GET_DOUBLE_PTR(eval_general, in, static_cast<int>(dims[0]), 0)
	
	plhs[0] = mxCreateDoubleMatrix(fun->NY(), 1, mxREAL); 
	out = mxGetPr(plhs[0] ); 
	
	fun->evaluate(out, in); // evaluate(TY & result, const TX & x)
	return;
	
      }
      else {
	if(dimsize > 2) mexErrMsgTxt("universalFunction:eval_general: Only two dimensions are supported.");
	int rows = dims[0];
	int cols = dims[1];
	int nx = fun->NX();
	int ny = fun->NY();
	if(fun->NX() != dims[0]){
	  mexErrMsgTxt("universalFunction:eval_general. Unexpected input dimension.");
	  return;
	}
	double* in;
	MEX_GET_DOUBLE_PTR(eval_piecewise, in, rows, 0)
	plhs[0] = mxCreateDoubleMatrix(fun->NY(), cols, mxREAL); 
	double *out = mxGetPr(plhs[0] ); 
		
	for(int i = 0; i < cols; i++){
	  fun->evaluate(out, in);
	  out += ny;
	  in += nx;	  
	}
	return;	
      }
    }
    
    
    if(!strcmp("getDefinitionInterval", cmd)) {
        MEX_CHECK(getDefinitionInterval, 0, 2, -1)
        functionLib::Function* fun = getFunctionPointer(prhs[1]);
        functionLib::PiecewiseFunction* fpw = dynamic_cast<functionLib::PiecewiseFunction*>(fun);
        if(fpw) {
            functionLib::Interval I = fpw->getDefinitionInterval();
            MEX_RETURN_DOUBLE(I.a, 0)
            MEX_RETURN_DOUBLE(I.b, 1)
            return;
        }
        
        functionLib::SingleVariableFunction* fsv = dynamic_cast<functionLib::SingleVariableFunction*>(fun);
        if(fsv) {
            double a = fsv->getT0();
            double b = fsv->getT0();
            MEX_RETURN_DOUBLE(a, 0)
            MEX_RETURN_DOUBLE(b, 1)
            return;
        }
        mexErrMsgTxt("universalFunction:getT0. Given input is not a single variable function.");
        return;        
    }
    
    if (!strcmp("getT0", cmd)) {
        MEX_CHECK(getT0, 0, 1, -1)
        functionLib::Function* fun = getFunctionPointer(prhs[1]);
        functionLib::SingleVariableFunction* f = dynamic_cast<functionLib::SingleVariableFunction*>(fun);
        if(f) {
            double d = f->getT0();
            MEX_RETURN_DOUBLE(d, 0)
        }
        else {
            mexErrMsgTxt("universalFunction:getT0. Given input is not a single variable function.");
        }
        return;
    }
    if (!strcmp("getTf", cmd)) {
        MEX_CHECK(getTf, 0, 1, -1)
        functionLib::Function* fun = getFunctionPointer(prhs[1]);
        functionLib::SingleVariableFunction* f = dynamic_cast<functionLib::SingleVariableFunction*>(fun);
        if(f) {
            double d = f->getTf();
            MEX_RETURN_DOUBLE(d, 0)
        }
        else {
            mexErrMsgTxt("universalFunction:getTf. Given input is not a single variable function.");
        }
        return;
    }
    
    
    if(!strcmp("copyWithCroppedOutput_piecewise", cmd)) {
        MEX_CHECK(copyWithCroppedOutput_piecewise, 0, 1, -1)
        functionLib::PiecewiseFunction* fun2 = convertMat2Ptr<functionLib::PiecewiseFunction>(prhs[1])->copyWithCroppedOutput();
        plhs[0] = convertPtr2Mat<functionLib::PiecewiseFunction>(fun2);
        return;
    }
    
    
    if(!strcmp("copyWithCroppedOutput_general", cmd)) {
        MEX_CHECK(copyWithCroppedOutput_general, 0, 1, -1)
        functionLib::Function* fun2 = convertMat2Ptr<functionLib::Function>(prhs[1])->copyWithCroppedOutput();
        plhs[0] = convertPtr2Mat<functionLib::Function>(fun2);
        return;        
    }
    
    
    if(!strcmp("copyWithDomainMapping_piecewise", cmd)){
        MEX_CHECK(copyWithDomainMapping_piecewise, 1, 1, -1)
        functionLib::PiecewiseFunction* f = convertMat2Ptr<functionLib::PiecewiseFunction>(prhs[1]);
        functionLib::PiecewiseFunction* fun2;
        if(nrhs == 2+1){
            double t0;
            MEX_GET_DOUBLE(copyWithDomainMapping_piecewise, t0, 0)
            fun2 = f->copyWithDomainMapping(t0);
        }
        else {
            double t0, tf;
            MEX_GET_DOUBLE(copyWithDomainMapping_piecewise, t0, 0)
            MEX_GET_DOUBLE(copyWithDomainMapping_piecewise, tf, 1)
            fun2 = f->copyWithDomainMapping(t0, tf);
        }
        plhs[0] = convertPtr2Mat<functionLib::PiecewiseFunction>(fun2);
        return;
    }
    
    
    if(!strcmp("copyWithDomainCut_piecewise", cmd)){
        MEX_CHECK(copyWithDomainCut_piecewise, 1, 1, -1)
        functionLib::PiecewiseFunction* f = convertMat2Ptr<functionLib::PiecewiseFunction>(prhs[1]);
        functionLib::PiecewiseFunction* fun2;
        if(nrhs == 2+1){
            double t0;
            MEX_GET_DOUBLE(copyWithDomainCut_piecewise, t0, 0)
            fun2 = f->copyWithDomainCut(t0);
        }
        else {
            double t0, tf;
            MEX_GET_DOUBLE(copyWithDomainCut_piecewise, t0, 0)
            MEX_GET_DOUBLE(copyWithDomainCut_piecewise, tf, 1)
            fun2 = f->copyWithDomainCut(t0, tf);
        }
        plhs[0] = convertPtr2Mat<functionLib::PiecewiseFunction>(fun2);
        return;
    }
    
    
    if(!strcmp("copyWithDomainShift_piecewise", cmd)){
        MEX_CHECK(copyWithDomainShift_piecewise, 1, 1, -1)
        double offset;
        MEX_GET_DOUBLE(copyWithDomainShift_piecewise, offset, 0)
        functionLib::PiecewiseFunction* fun2 = convertMat2Ptr<functionLib::PiecewiseFunction>(prhs[1])->copyWithDomainShift(offset);
        plhs[0] = convertPtr2Mat<functionLib::PiecewiseFunction>(fun2);
        return;
    }
    
    
    MEX_CHECK(serialize, 0, 100, -1)
    functionLib::Function* _fun = getFunctionPointer(prhs[1]);
    
    
    
    if(!strcmp("serialize", cmd)){
      MEX_CHECK(serialize, 0, 1, -1)
      if(!_fun){
	mexErrMsgTxt("Cannot serialize function.");
      }
      ostringstream stream;
      ofc::functionLib::Function::serialize(_fun, stream);
      MEX_RETURN_STRING(stream.str(), 0)
      return;
    }
    
    MEX_FUN_GETTER_DOUBLE(_fun, NX)
    
    MEX_FUN_GETTER_DOUBLE(_fun, NY) 
    
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}


void sortLinearData(std::vector<double> &data, std::vector<double> *times, std::vector<double> *coefficients, int n){
  for(int i = 0; i < data.size(); i++){
    if(i % (n+1) == 0){
      times->push_back(data.at(i));
    } else {
      coefficients->push_back(data.at(i));
    }
  }
}

void sortCubicData(std::vector<double> &data, std::vector<double> *times, std::vector<double> *coefficients, int n){
  int i;
  for(i = 0; i < data.size()-n-1; i++){
    if( i % (4*n+2) < 2 ){
      times->push_back(data.at(i));
    } else {
      coefficients->push_back(data.at(i));
    }
  }
  times->push_back(data.at(i++));
  while(i < data.size())
    coefficients->push_back(data.at(i++));
}

void parseAndStoreTrajectory(std::istream &stream, mxArray** mxpointer, int index){
  functionLib::PiecewiseFunction* traj;
  functionLib::TrajectoryReader reader;
  bool succ = reader.read(stream, &traj);
  if(succ){
    mxpointer[index] = convertPtr2Mat<functionLib::PiecewiseFunction>(traj);
    return;	
  } else {
    mexErrMsgTxt("Unable to parse trajectory information.\n");
    return;
  }
}

functionLib::Function* getFunctionPointer(const mxArray* mxptr){
  functionLib::Function* f = 0;
  f = tryConvertMat2Ptr<functionLib::PiecewiseFunction>(mxptr);
  if(!f){
    f = tryConvertMat2Ptr<functionLib::Function>(mxptr);
  }
  if(!f){
    f = tryConvertMat2Ptr<functionLib::Polynom>(mxptr);
  }
  if(!f){
    f = tryConvertMat2Ptr<functionLib::NormalizedPolynom>(mxptr);
  }
  return f;
}

