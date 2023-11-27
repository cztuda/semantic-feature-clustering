
#include "mex.h"

#include <string>
#include <map>
#include <cstdio>
#include <cstring>
#include <iostream>

#define printfFnc(...) { mexPrintf(__VA_ARGS__); mexEvalString("drawnow;");}


#define MEX_ERROR(NAME, TEXT, ...)                              \
{                                                               \
  const size_t len = 100;                                       \
  char tmp[len];                                                \
  snprintf(tmp, len, #TEXT, #NAME, ##__VA_ARGS__);              \
  mexErrMsgTxt(tmp);                                            \
}

#define MEX_GET_STRING(NAME, VARIABLE, INDEX) 					\
{                                                               \
  if(!mxIsChar(prhs[INDEX])) {                                  \
    if(mxIsClass(prhs[INDEX], "string")) {                      \
      MEX_ERROR(NAME, "%s: Input %i must be a char-array string. So use '' instead of """, INDEX);}	\
    else {                                                      \
      MEX_ERROR(NAME, "%s: Input %i must be a string.", INDEX);}\
  }                                                             \
  mwSize buflen = mxGetN(prhs[INDEX])*sizeof(mxChar)+1;			\
  char* c_array = new char[buflen];                             \
  int status = mxGetString(prhs[INDEX],c_array,buflen);			\
  VARIABLE = std::string(c_array);                              \
  delete [] c_array;                                            \
}

#define MEX_READ_STRING(NAME, VARIABLE, MXARRAY) 				\
  if(mxIsChar(MXARRAY)){                                        \
      mwSize buflen = mxGetN(MXARRAY)*sizeof(mxChar)+1;			\
      char* c_array = new char[buflen];                         \
      int status = mxGetString(MXARRAY,c_array,buflen);			\
      VARIABLE = std::string(c_array);                          \
      delete [] c_array;                                        \
  } else {                                                      \
      VARIABLE = "";                                            \
  }

#define MEX_FILL_CHAR_ARRAY(VARIABLE, VARLEN, MXARRAY, NWRITTEN)\
{                                                               \
  NWRITTEN = 0;                                                 \
  if(mxIsChar(MXARRAY)){                                        \
      mwSize buflen = mxGetN(MXARRAY)*sizeof(mxChar)+1;			\
      NWRITTEN = buflen;                                        \
      if(buflen > VARLEN){                                      \
          NWRITTEN = VARLEN;                                    \
      }                                                         \
      memcpy(VARIABLE, mxGetPr(MXARRAY), NWRITTEN*sizeof(char));\
  }                                                             \
}

void printDocumentation(const mxArray *args[] , int nargin);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    
    int nargin = nrhs;
    int nargout = nlhs;
    const mxArray** args = prhs;
    
    if(nrhs <= 1) {
        return;
    }
    
    bool noDefault = (nrhs % 2 != 0); // noDefault = WENN(ungerade Anz Argumente)
    // odd number of arguments -> second input must be a string
    if(nrhs % 2 != 0 && !mxIsChar(prhs[1])) {
        MEX_ERROR(getFunctionArguments, "For odd number of input arguments, the second input must be a string.");
    }
    // even number of arguments -> second input must be a cell or empty
    if(nrhs % 2 == 0 && !(mxIsCell(prhs[1]) || mxIsEmpty(prhs[1]))) {
        MEX_ERROR(getFunctionArguments, "For even number of input arguments, the second input must be an (empty) cell.");
    }
	
  if(nlhs == 0){
    // print documentation
    printDocumentation(prhs, nrhs);
    return ; 
  }
  
  
  // forward default arguments to output
  size_t outPos = 0;
  size_t n_unlabelled;
  int labelStartIndex = 2;
  if(!noDefault){
      n_unlabelled = mxGetNumberOfElements(args[1]);
      if(!mxIsEmpty(args[1]) && mxIsCell(args[1])) {
          for(size_t i = 0; i < n_unlabelled; i++){ 
              plhs[outPos++] = mxDuplicateArray(mxGetCell(args[1],i));        
          }
      }
  }
  else {
      n_unlabelled = 0;
      labelStartIndex = 1;
  }
  
  std::map<std::string, int> ATI;

  int pos = outPos;
  // read arguments and their default values:
  for(int i = labelStartIndex; i < nargin; i+=2){ 
    std::string s;
    MEX_GET_STRING(getFunctionArguments, s, i)
    
    // store default argument in retval
    plhs[outPos++] = mxDuplicateArray(prhs[i+1]);
    ATI.insert(std::make_pair(s, pos++));
    
  }
  if(outPos != nargout){
    MEX_ERROR(getFunctionArguments,  "Invalid number of output arguments.");
  }
  
  // check input for argument strings
  if(! mxIsCell(args[0]) && !mxIsEmpty(args[0])){
    MEX_ERROR(getFunctionArguments, "Interner Fehler");
  }
  
  mxArray* IN = mxDuplicateArray(prhs[0]);
  size_t n_IN =  mxGetNumberOfElements(IN);
  
  bool in_labelled = (noDefault);
  std::string s;
  outPos = 0;
  
  // go through all user input and decide if it is unlabelled or labelled input
  for(int i = 0; i < n_IN; i++){
    
    mxArray* elt = mxGetCell(IN, i);
    
    if(i < n_unlabelled && ! in_labelled){ // as long as there are still unlabelled arguments and we did not start to parse labelled input arguments
        if(mxIsChar(elt)) { // labelled input starts with a char array
            MEX_READ_STRING(getFunctionArguments, s, elt)
            auto pair = ATI.find(s); // try to find input in argument list
            if(pair != ATI.end() && i+1 < n_IN){ // entry found, so start with labelled input
                plhs[pair->second] = mxDuplicateArray(mxGetCell(IN, ++i));
                in_labelled = true;
            }
            else { // label not found, so use as unlabelled input and hope that the user did not misspell a label
                plhs[outPos++] = mxDuplicateArray(elt);
            }
        }
        else { // not a char, so definitively unlabelled
            plhs[outPos++] = mxDuplicateArray(elt);
        }
    }
    else { // started to parse labelled input, so this and all following input is labelled
        if(!mxIsChar(elt)){
          continue;
        }
        MEX_READ_STRING(getFunctionArguments, s, elt)
        // get argument position in retval and replace the default value by the user input
        auto pair = ATI.find(s);
        if(pair != ATI.end() && i+1 < n_IN){
          plhs[pair->second] = mxDuplicateArray(mxGetCell(IN, ++i));
        }        
    }
    
  } // end for
  
  return;
  
}



void printDocumentation(const mxArray *prhs[] , int nargin) {
  // print documentation
  printfFnc("% OPTIONAL ARGUMENTS\n");
  for(int i = 2; i < nargin; i+=2){
    if(mxIsChar(prhs[i])){
      std::string ch;
      MEX_GET_STRING(getFunctionArguments, ch, i)
      
      char buff[16];
      for(int i = 0; i < 16 && i < ch.size(); i++){
           buff[i] = ch[i];
      }
      
      char res[58];
      
      res[0] = ' ';
      res[1] = '\0';
      
//       if(mxIsChar(prhs[i+1])){
//           res[0] = '\'';
//           size_t n = 0;
//           MEX_FILL_CHAR_ARRAY(&res[1], 55, prhs[i+1], n)
//           res[n+1] = '\'';
//           res[n+2] = '\0';
//       }
//       else {
//           mxArray * in = mxDuplicateArray(prhs[i+1]);
//           mxArray *out[1];
//           mxArray *out2[1];
//           bool failure = mexCallMATLAB(1, out, 1, &in, "mlreportgen.utils.toString");
//           if(failure){
//               std::cout << "failure1" << std::endl;
//               continue;
//           }
//           failure = mexCallMATLAB(1, out2, 1, out, "char");
//           if(failure){
//               std::cout << "failure2" << std::endl;
//               continue;
//           }
//           size_t n = 0;
//           MEX_FILL_CHAR_ARRAY(res, 57, out2[1], n)
//           res[n] = '\0';
//       }
//       if(prhs(i+1).is_string()){
//         int l = getCString(prhs(i+1), &res[1], 58);
//         res[0] = '\'';
//         if(l > 55)
//           l = 55;
//         res[l+1] = '\'';
//         res[l+2] = '\0';
//       }
//       else if(prhs(i+1).is_function()  || prhs(i+1).is_function_handle()) {
//         strcpy(res, "function_handle");
//       }
//       else if(prhs(i+1).is_object()){
//         strcpy(res, "<object ");
//         int len = getCString(feval("class", prhs(i+1), 1)(0), &res[8], 48);
//         res[8+len] = '>';
//         res[8+len+1] = '\0';
//       }
//       else{
//         getCString(feval ("mat2str", prhs(i+1), 1)(0), res, 58);
//       }
      printfFnc("%%   %s: %s\n", buff, res);
    }else {
      MEX_ERROR(getFunctionArguments, "Invalid input argument.")
    }
  }
}

