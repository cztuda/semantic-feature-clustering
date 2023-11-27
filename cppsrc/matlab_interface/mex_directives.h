#pragma once

#ifndef MEX_DIRECTIVES_H
#define MEX_DIRECTIVES_H


#include "mex.h"
#include "../math/definitions.h"

#include <cstdio>


/*
 * DATA TYPE ALIAS
 */

#define MEX_DOUBLE 1
#define MEX_VECTORX 2
#define MEX_MATRIXx 3


/*
 * Utility and debug
 */
#define printfFnc(...) { mexPrintf(__VA_ARGS__); mexEvalString("drawnow;");}


#define printInputInformation(NAME) \
printfFnc(#NAME); \
printfFnc(". num in=%i\n", nrhs); \
    for(int i = 0; i < nrhs; i++){ \
      if(mxIsDouble(prhs[i]) && mxGetNumberOfElements(prhs[i]) == 1){ \
	printfFnc("input %i, double: %f\n", i, mxGetScalar(prhs[i])); \
      } \
      if(mxIsDouble(prhs[i]) && mxGetNumberOfElements(prhs[i]) > 1){ \
	printfFnc("input %i, double[], numel=%i, first: %f\n", i, mxGetNumberOfElements(prhs[i]) , mxGetScalar(prhs[i])); \
      } \
      if(mxIsChar(prhs[i])){ \
	printfFnc("input %i, string: %s\n", i, mxArrayToString(prhs[i])); \
      } \
    } \


/*
 * FUNCTION PARTS
 */



template<typename... Args>
inline void mex_error(const std::string& name, const std::string& msg, Args... args){
    const size_t len = 1024;
    char tmp[len];
    std::snprintf(tmp, len, msg.c_str(), name.c_str(), args...); 
    mexErrMsgTxt(tmp);
}

#define MEX_ERROR(NAME, TEXT, ...) \
{ \
  mex_error(#NAME, #TEXT, ##__VA_ARGS__);  \
}

#define MEX_CHECK(NAME, N_IN_REQ, N_OUT_MAX, N_OUT_MIN) \
  if (nlhs < 0 || nrhs < N_IN_REQ + 2){ \
    MEX_ERROR(NAME, "%s: Additional argument(s) required.") \
  } \
  if(nlhs > N_OUT_MAX) { \
    MEX_ERROR(NAME, "%s: Too many output arguments.") \
  } \
  if(nlhs < N_OUT_MIN && N_OUT_MIN >= 0) { \
    MEX_ERROR(NAME "%s: Expected at least %i output arguments.", N_OUT_MIN); \
  }

    
// store double VALUE in return value INDEX
#define MEX_RETURN_DOUBLE(VALUE, INDEX) \
  if(nlhs > INDEX) {\
    plhs[INDEX] = mxCreateDoubleScalar(VALUE); \
  }

// store [] in return value INDEX
#define MEX_RETURN_EMPTY(INDEX) \
  if(nlhs > INDEX) {\
    plhs[INDEX] = mxCreateDoubleMatrix( 0, 0, mxREAL ); \
  }
  
  
// get vector from some 
// container DATA
// and export to return value INDEX
#define MEX_RETURN_VECTOR(DATA, INDEX) \
  if(nlhs > INDEX) { \
    int l = DATA.size(); \
    plhs[INDEX] = mxCreateDoubleMatrix(l, 1, mxREAL); \
    double* data = mxGetPr(plhs[INDEX]); \
    std::copy(DATA.begin(), DATA.end(), data); \
  }
  
// get vector from 
// void FUN_NAME(TVectorX &var)
// and export to return value INDEX
#define MEX_RETURN_VECTOR_X(OBJECT, FUN_NAME, INDEX) \
  if(nlhs > INDEX) { \
    TVectorX vec; \
    OBJECT->FUN_NAME(vec); \
    EigenVectorToMxArray(&plhs[INDEX], vec, true); \
  }
  
// get vector from 
// TYPE FUN_NAME()
// cast each value to double
// and export to return value INDEX
#define MEX_RETURN_OBJVECTOR_X_(OBJECT, FUN_NAME, TYPE, INDEX) \
  if(nlhs > INDEX) { \
    const TYPE* vec = OBJECT->FUN_NAME(); \
    int n = vec->size(); \
    plhs[INDEX] = mxCreateDoubleMatrix( n, 1, mxREAL); \
    double* mxdata = mxGetPr(plhs[INDEX]); \
    for(int i = 0; i < n; i++) \
      mxdata[i] = (double)(*vec)(i); \
  }  
  
// get vector from 
// void FUN_NAME(TYPE& vec)
// cast each value to double
// and export to return value INDEX
#define MEX_RETURN_OBJVECTOR_X(OBJECT, FUN_NAME, TYPE, INDEX) \
  if(nlhs > INDEX) { \
    TYPE vec; \
    OBJECT->FUN_NAME(vec); \
    int n = vec.size(); \
    plhs[INDEX] = mxCreateDoubleMatrix( n, 1, mxREAL); \
    double* mxdata = mxGetPr(plhs[INDEX]); \
    for(int i = 0; i < n; i++) \
      mxdata[i] = (double)vec(i); \
  }  
  

// get double array of length LEN from 
// void FUN_NAME(double *var)
// and export to return value INDEX
#define MEX_RETURN_DOUBLE_PTR(FUN_NAME, LEN, INDEX) \
  if(nlhs > INDEX) { \
    plhs[INDEX] = mxCreateDoubleMatrix(LEN, 1, mxREAL); \
    double *mxdata = mxGetPr(plhs[INDEX] ); \
    FUN_NAME(mxdata); \
  }

// get double array of length LEN from 
// void FUN_NAME(double *var)
// and copy to return value INDEX
#define MEX_RETURN_DOUBLE_PTR_COPYDATA(FUN_NAME, LEN, INDEX) \
  if(nlhs > INDEX) { \
    plhs[INDEX] = mxCreateDoubleMatrix(LEN, 1, mxREAL); \
    double *mxdata = mxGetPr(plhs[INDEX] ); \
    double *outdata; \
    FUN_NAME(outdata); \
    memcpy(mxdata, outdata, LEN*sizeof(double)); \
  }
 
// get double array of length LEN from 
// const double * FUN_NAME()
// and copy to return value INDEX
#define MEX_RETURN_DOUBLE_PTR2_COPYDATA(FUN_NAME, LEN, INDEX) \
  if(nlhs > INDEX) { \
    plhs[INDEX] = mxCreateDoubleMatrix(LEN, 1, mxREAL); \
    double *mxdata = mxGetPr(plhs[INDEX] ); \
    const double *outdata = FUN_NAME(); \
    memcpy(mxdata, outdata, LEN*sizeof(double)); \
  }
  
  
// copy the std::string STR
// to the return value INDEX
#define MEX_RETURN_STRING(STR, INDEX) \
  if(nlhs > INDEX) { \
    int l = STR.size(); \
    char* buffer = (char*) mxCalloc(l+1, sizeof(char));\
    memcpy(buffer, STR.data(), sizeof(char)*l); \
    buffer[l] = '\0'; \
    plhs[INDEX] = mxCreateString(buffer); \
  }

// copy the char* STR 
// to the return value INDEX
#define MEX_RETURN_CSTRING(STR, INDEX) \
  if(nlhs > INDEX) { \
    int len = strlen(STR); \
    char* buffer = (char*) mxCalloc(len+1, sizeof(char));\
    memcpy(buffer, STR, sizeof(char)*(len+1)); \
    plhs[INDEX] = mxCreateString(buffer); \
  }
  
// call get the vector at prhs[INDEX+2] and save in TVectorX VARIABLE
#define MEX_GET_TVectorX(NAME, VARIABLE, INDEX) \
{ \
  if(!isVector(prhs[INDEX+2])) \
    MEX_ERROR(NAME, "%s: Input %i must be a row or column vector.", INDEX+1); \
  int len = mxGetNumberOfElements(prhs[INDEX + 2]); \
  CppArrayToEigenVector(mxGetPr(prhs[INDEX + 2]), len, VARIABLE); \
}
  
// get the double value from input at INDEX
#define MEX_GET_DOUBLE(NAME, VARIABLE, INDEX) \
{ \
  if(!mxIsDouble(prhs[INDEX+2]) || mxGetNumberOfElements(prhs[INDEX+2]) != 1) \
    MEX_ERROR(NAME, "%s: Input %i must be a scalar double value.", INDEX+1); \
  VARIABLE = mxGetScalar(prhs[INDEX+2]); \
}

#define MEX_GET_DOUBLE_PTR(NAME, VARIABLE, LEN, INDEX) \
{ \
  if(!mxIsDouble(prhs[INDEX+2])) \
    MEX_ERROR(NAME, "%s: Input %i must be a double value.", INDEX+1); \
  if(LEN >= 0 && mxGetNumberOfElements(prhs[INDEX+2]) != LEN) \
    MEX_ERROR(NAME, "%s: Unexpected dimension of input vector %i (given %zu, but expected %i).", INDEX+1, mxGetNumberOfElements(prhs[INDEX+2]), LEN); \
  VARIABLE = mxGetPr(prhs[INDEX+2]); \
}

#define MEX_GET_STD_VECTOR_DOUBLE(NAME, VARIABLE, INDEX)    \
{                                                           \
    if(!mxIsDouble(prhs[INDEX+2]))                          \
        MEX_ERROR(NAME, "%s: Input %i must be a double value.", INDEX+1); \
    int len = mxGetNumberOfElements(prhs[INDEX + 2]);       \
    double* p = mxGetPr(prhs[INDEX+2]);                     \
    VARIABLE = std::vector<double>(p, p+len);               \
}


#define MEX_GET_STRING(NAME, VARIABLE, INDEX) 					\
{										\
  if(!mxIsChar(prhs[INDEX+2])) 							\
    MEX_ERROR(NAME, "%s: Input %i must be a string.", INDEX+1); 		\
  mwSize buflen = mxGetN(prhs[INDEX+2])*sizeof(mxChar)+1;			\
  char* c_array = new char[buflen];						\
  mxGetString(prhs[INDEX+2],c_array,buflen);			\
  VARIABLE = std::string(c_array);						\
  delete [] c_array;								\
}


#define MEX_PARSE_AND_RETURN_PIECEWISE_FUNCTION(DESCRIPTION, INDEX)			\
  functionLib::PiecewiseFunction* traj;							\
  functionLib::TrajectoryReader reader;							\
  std::istringstream is(DESCRIPTION);							\
  reader.read(is, &traj);								\
  if(nlhs > INDEX){									\
    plhs[INDEX] = convertPtr2Mat<functionLib::PiecewiseFunction>(traj);			\
  }
  
/*
 * COMPLETE FUNCTIONS
 */

//void function();
#define MEX_FUN_CALL(OBJ, NAME) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 0, -1) \
    OBJ->NAME(); \
    return; \
  }

  ///// GETTER /////

  // double obj->getterfun();
#define MEX_FUN_GETTER_DOUBLE(OBJ, NAME) \
  if (!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 1, -1) \
    double d = OBJ->NAME(); \
    MEX_RETURN_DOUBLE(d, 0) \
    return; \
  }
  
  // string obj->getterfun();
#define MEX_FUN_GETTER_STRING(OBJ, NAME) \
  if (!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 1, -1) \
    std::string s = OBJ->NAME(); \
    MEX_RETURN_STRING(s, 0) \
    return; \
  }
  
  // (double) retval = int obj->getterfun();
#define MEX_FUN_GETTER_TODOUBLE(OBJ, NAME) \
  if (!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 1, -1) \
     \
    MEX_RETURN_DOUBLE((double)OBJ->NAME(), 0) \
    return; \
  }
  
  
  // void obj->getterfun(double &val);
#define MEX_FUN_GETTER_DOUBLE_(OBJ, NAME) \
  if (!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 1, -1) \
    double val; \
    OBJ->NAME(val); \
    MEX_RETURN_DOUBLE(val, 0) \
    return; \
  }
  
  
// TYPE var
// success obj->NAME(var);
// plhs[INDEX] = var;
#define MEX_FUN_GETTER_OBJECT(OBJ, NAME, TYPE, INDEX) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 1, -1) \
    TYPE *var; \
    OBJ->NAME(var); \
    if(nlhs > INDEX) \
      plhs[INDEX] = convertPtr2Mat<TYPE>(var); \
    return; \
  }
  
  
//
// TYPE var = obj->NAME();
// plhs[INDEX] = var;
#define MEX_FUN_GETTER_OBJECT_(OBJ, NAME, TYPE, INDEX) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 1, -1) \
    TYPE* var = OBJ->NAME(); \
    if(nlhs > INDEX) \
      plhs[INDEX] = convertPtr2Mat<TYPE>(var); \
    return; \
  }

  
// void functionname(TVectorX &out);
#define MEX_FUN_GETTER_TVECTORX(OBJ, NAME) \
  if (!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 1, -1) \
     \
    MEX_RETURN_VECTOR_X(OBJ, NAME, 0) \
    return; \
  }  
  
  
// void functionname(TYPE &out);
#define MEX_FUN_GETTER_OBJVECTORX(OBJ, NAME, TYPE) \
  if (!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 1, -1) \
     \
    MEX_RETURN_OBJVECTOR_X(OBJ, NAME, TYPE, 0) \
    return; \
  } 
  
#define MEX_FUN_GETTER_ENUMOPTION(OBJ, NAME) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 0, 1, -1) \
    MEX_RETURN_STRING(OBJ->NAME(), 0) \
    return; \
  }
  
  ///// SETTER /////
  
// void obj->setterfun(prhs[0]);
#define MEX_FUN_SETTER_DOUBLE(OBJ, NAME) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 1, 0, -1) \
    double var; \
    MEX_GET_DOUBLE(NAME, var, 0) \
    OBJ->NAME(var); \
    return; \
  }
  
// void obj->setterfun(prhs[0]);
#define MEX_FUN_SETTER_STRING(OBJ, NAME) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 1, 0, -1) \
    std::string var; \
    MEX_GET_STRING(NAME, var, 0) \
    OBJ->NAME(var); \
    return; \
  }
  
// bool obj->setterfun(prhs[0]);
#define MEX_FUN_SETTER_STRING_WRETURN(OBJ, NAME) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 1, 1, -1) \
    std::string var; \
    MEX_GET_STRING(NAME, var, 0) \
    bool succ = OBJ->NAME(var); \
    MEX_RETURN_DOUBLE(succ, 0); \
    return; \
  }
  
// void obj->setterfun((TYPE)prhs[0]);
#define MEX_FUN_SETTER_TODOUBLE(OBJ, NAME, TYPE) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 1, 0, -1) \
    double var; \
    MEX_GET_DOUBLE(NAME, var, 0) \
    OBJ->NAME((TYPE)var); \
    return; \
  }

#define MEX_FUN_SETTER_OBJVECTOR_X(OBJECT, NAME, TYPE) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 1, 0, -1) \
    double* var; \
    if(!mxIsDouble(prhs[0+2])) \
      MEX_ERROR(NAME, "%s: Input %i must be a double value.", 1); \
    var = mxGetPr(prhs[0+2]); \
    TYPE val; \
    val.resize(mxGetNumberOfElements(prhs[0+2])); \
    for(int i = 0; i < val.size(); i++) \
      val[i] = var[i]; \
    OBJECT->NAME(&val); \
    return; \
  }
  
#define MEX_FUN_SETTER_OBJVECTOR_X_REF(OBJECT, NAME, TYPE) \
  if(!strcmp(#NAME, cmd)) { \
    MEX_CHECK(NAME, 1, 0, -1) \
    double* var; \
    if(!mxIsDouble(prhs[0+2])) \
      MEX_ERROR(NAME, "%s: Input %i must be a double value.", 1); \
    var = mxGetPr(prhs[0+2]); \
    TYPE val; \
    val.resize(mxGetNumberOfElements(prhs[0+2])); \
    for(int i = 0; i < val.size(); i++) \
      val[i] = var[i]; \
    OBJECT->NAME(val); \
    return; \
  }
 
  
#define MEX_FUN_SETTER_ENUMOPTION(OBJECT, NAME) \
  if(!strcmp(#NAME, cmd)) { \
      MEX_CHECK(NAME, 1, 0, -1) \
      std::string s; \
      MEX_GET_STRING(NAME, s, 0) \
      bool b = OBJECT->NAME(s); \
      MEX_RETURN_DOUBLE(b, 0) \
      return; 		\
    }
  
  
  ///// PROPERTY GETTER/SETTER /////
  
#define MEX_FUN_GETTERSETTER_DOUBLE(OBJ, NAME)   \
  MEX_FUN_GETTER_DOUBLE(OBJ, get ## NAME) \
   \
   MEX_FUN_SETTER_DOUBLE(OBJ, set ## NAME)
   
     
#define MEX_FUN_GETTERSETTER_INT(OBJ, NAME)   \
  MEX_FUN_GETTER_TODOUBLE(OBJ, get ## NAME) \
   \
   MEX_FUN_SETTER_TODOUBLE(OBJ, set ## NAME, int)
     
     
#define MEX_FUN_GETTERSETTER_BOOL(OBJ, NAME)   \
  MEX_FUN_GETTER_TODOUBLE(OBJ, get ## NAME) \
   \
   MEX_FUN_SETTER_TODOUBLE(OBJ, set ## NAME, bool)

   
#define MEX_FUN_GETTERSETTER_CUSTOM(OBJ, NAME, CUSTOMTYPE)   \
  MEX_FUN_GETTER_TODOUBLE(OBJ, get ## NAME) \
   \
   MEX_FUN_SETTER_TODOUBLE(OBJ, set ## NAME, CUSTOMTYPE)

   
#define MEX_FUN_GETTERSETTER_OBJVECTOR_X(OBJ, NAME, TYPE) \
  MEX_FUN_GETTER_OBJVECTORX(OBJ, get ## NAME, TYPE) \
   \
  MEX_FUN_SETTER_OBJVECTOR_X(OBJ, set ## NAME, TYPE)
  
#define MEX_FUN_GETTERSETTER_TVECTOR_X_REF(OBJ, NAME) \
  MEX_FUN_GETTER_TVECTORX(OBJ, get ## NAME) \
   \
  MEX_FUN_SETTER_OBJVECTOR_X_REF(OBJ, set ## NAME, TVectorX)
  
#define MEX_FUN_GETTERSETTER_OBJVECTOR_X_REF(OBJ, NAME, TYPE) \
  MEX_FUN_GETTER_OBJVECTORX(OBJ, get ## NAME, TYPE) \
   \
  MEX_FUN_SETTER_OBJVECTOR_X_REF(OBJ, set ## NAME, TYPE)
   
   
#define MEX_FUN_GETTERSETTER_ENUMOPTION(OBJ, NAME) \
  MEX_FUN_GETTER_ENUMOPTION(OBJ, get ## NAME)\
  \
  MEX_FUN_SETTER_ENUMOPTION(OBJ, set ## NAME)

#define MEX_ATTRIBUTE_GETTERSETTER_NUMBER(OBJ, NAME, TYPE) \
    if(!strcmp(#NAME, cmd)) { \
        if(nrhs > 2) { \
            double num; \
            MEX_GET_DOUBLE(NAME, num, 0) \
            OBJ->NAME = (TYPE)num; \
        } \
        if(nlhs > 0) { \
            if(nlhs > 1) \
                MEX_ERROR(NAME, "%s: Too many output arguments.") \
            MEX_RETURN_DOUBLE((double)OBJ->NAME, 0) \
        } \
        return; \
    }

  
#define MEX_ATTRIBUTE_GETTERSETTER_STRING(OBJ, NAME) \
    if(!strcmp(#NAME, cmd)) { \
        if(nrhs > 2) { \
            mwSize buflen = mxGetN(prhs[2])*sizeof(mxChar)+1; \
            char* c_array = new char[buflen]; \
            mxGetString(prhs[2],c_array,buflen); \
            OBJ->NAME = std::string(c_array); \
            delete [] c_array; \
        } \
        if(nlhs > 0) { \
            if(nlhs > 1) \
                MEX_ERROR(NAME, "%s: Too many output arguments.") \
            int l = OBJ->NAME.size(); \
            char* buffer = (char*) mxCalloc(l+1, sizeof(char));\
            memcpy(buffer, OBJ->NAME.data(), sizeof(char)*l); \
            buffer[l] = '\0'; \
            plhs[0] = mxCreateString(buffer); \
        } \
        return; \
    }
  
  
/*
 * Useful functions,
 * not scripted
 */


inline void mxArrayToEigenMatrix(const mxArray* array, Eigen::MatrixXd& matrix){
      int m = mxGetM(array);
      int n = mxGetN(array);
      double* data = mxGetPr(array);
      
      matrix = Eigen::Map<Eigen::MatrixXd>(data, m, n);
}
inline void mxArrayToEigenVector(const mxArray* array, Eigen::VectorXd& vector){
      int m = mxGetM(array);
      int n = mxGetN(array);
      double* data = mxGetPr(array);
      
      vector = Eigen::Map<Eigen::VectorXd>(data, (m>1) ? m : n);
}


void CppArrayToEigenVector(double* array, int alength, ofc::math::TVectorX &matrix) {
  matrix = Eigen::Map<ofc::math::TVectorX>(array, alength);
}

void EigenVectorToMxArray(mxArray** array, const ofc::math::TVectorX &matrix, bool createNew=false){
  int n = matrix.rows();
  if(createNew){
    *array = mxCreateDoubleMatrix( n, 1, mxREAL);
  }
  double* mxdata = mxGetPr(*array);
  for(int i = 0; i < n; i++)
    mxdata[i] = matrix[i];
}
  

bool isVector(const mxArray *array){
  int ndim = mxGetNumberOfDimensions(array);
  if(ndim > 1){
    const mwSize* dim = mxGetDimensions(array);
    int counter = 0;
    for(int i = 0; i < ndim; i++){
      if(dim[i] > 1) ++counter;
    }
    return (counter < 2);
  }
  return true;
}


bool isRealNonemptyVector(const mxArray *array) {    
    return isVector(array) && mxIsNumeric(array) && !mxIsComplex(array) && !mxIsEmpty(array);
}


#define MEX_RETURN_DMATRIX(DMATRIX, INDEX) 			\
    double* data = DMATRIX.GetPr();				\
    int m = DMATRIX.getn(); 					\
    int n = DMATRIX.getm(); 					\
    plhs[INDEX] = mxCreateDoubleMatrix( m, n, mxREAL );		\
    double* mxdata = mxGetPr(plhs[INDEX]);			\
    memcpy(mxdata, data, m*n*sizeof(double));			
  

template<typename T>    
inline void MEX_RETURN_DOUBLE_PTR_COPYFROM(mxArray *plhs[], const int nlhs, const int index, const T& container){
    if(index < nlhs){
        int n = container.size();
        plhs[index] = mxCreateDoubleMatrix( n, 1, mxREAL );
        double* mxdata = mxGetPr(plhs[index]);
        int i = 0;
        for(auto it = container.begin(); it < container.end(); ++it){
            mxdata[i++] = *it;
        }
    }
}    
    
  
#endif // MEX_DIRECTIVES_H
