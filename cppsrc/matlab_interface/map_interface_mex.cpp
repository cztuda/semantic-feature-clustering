#include "mex.h"

#include "class_handle.hpp"
#include "../dlc/dlclass.hpp"
#include "mex_directives.h"

#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <memory>
#include <boost/serialization/map.hpp>

typedef std::map<std::string, double> t_map;

void getInput(const mxArray** prhs, double** value_ptr, int& n, std::vector<std::string>& keys);

inline void getString(const mxArray* A, std::string& s, const unsigned int BUFLEN=64){
    if(!mxIsChar(A)){
        mexErrMsgTxt("Given input is not a string.");
    }
    if(mxGetNumberOfElements(A) > BUFLEN-1){
        mexErrMsgTxt(std::string("New: The  char array in each cell must not be larger than ").append(std::to_string(BUFLEN-1)).append(" characters.").c_str());
    }
    std::vector<char> buffer(BUFLEN);
    char* c_array = buffer.data();
    mxGetString(A, c_array, BUFLEN);
    s = std::string(c_array);
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    
    // Get the command string
    char cmd[64];
    if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
        mexErrMsgTxt("First input should be a command string less than 64 characters long.");
    
    
    //////////////////////////////////////////////
    /////////////// Static methods ///////////////
    //////////////////////////////////////////////
    
    if(!strcmp("unserialize", cmd)) {
        mexErrMsgTxt("unserialize: Not yet implemented.");
        return;
    }
    
    //////////////////////////////////////////////
    //////// Constructor and desctructors ////////
    //////////////////////////////////////////////
    
    // Call constructor
    if (!strcmp("new", cmd)) {
        
        if (nlhs < 1)
            mexErrMsgTxt("New: One output expected.");
        if(nrhs > 1 && nrhs != 3){
            mexErrMsgTxt("New: If additiona input arguments are given, there must be exactly two input arguments.");
        }
        
        int n = 0;
        double* value_ptr = nullptr;
        std::vector<std::string> keys;
        
        if(nrhs > 1){
            getInput(&prhs[1], &value_ptr, n, keys);
        }
        
        t_map* mapobj = new t_map();
        
        if(value_ptr != nullptr){
            for(int i = 0; i < n; ++i){
                mapobj->operator[](keys[i]) = value_ptr[i];
            }
        }
        
        plhs[0] = convertPtr2Mat<t_map>(mapobj);
        return;
    }
    
    // Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
        mexErrMsgTxt("Second input should be a class instance handle.");
    
    // Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<t_map>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    t_map *mapobj = convertMat2Ptr<t_map>(prhs[1]);
    
    if(!strcmp("copy", cmd)){
        t_map* mapobj2 = new t_map(*mapobj);
        plhs[0] = convertPtr2Mat<t_map>(mapobj2);
        return;
    }
    
    
    
    ///////////////////////////////////////////////
    //////////// Various class methods ////////////
    ///////////////////////////////////////////////
    
    
    if (!strcmp("insert", cmd)) {
        MEX_CHECK(insert, 2, 0, -1)
        double* value_ptr = nullptr;
        int n;
        std::vector<std::string> keys;
        getInput(&prhs[2], &value_ptr, n, keys);
        
        if(value_ptr != nullptr){
            for(int i = 0; i < n; ++i){
                mapobj->operator[](keys[i]) = value_ptr[i];
            }
        }  
        
        return;
    }
    
    if (!strcmp("remove", cmd)) {
        MEX_CHECK("remove", 1, 0, -1)
        std::string s;
        getString(prhs[2], s);
        mapobj->erase(s);
        
        return;
    }
    
    if (!strcmp("lookup", cmd)) {
        MEX_CHECK("lookup", 1, 1, -1)
        std::string in;
        if(mxIsCell(prhs[2])){
            mxArray* cptr;
            std::string key;
            int n = mxGetNumberOfElements(prhs[2]);
            std::vector<double> values;
            
            for(int i = 0; i < n; ++i){
                cptr = mxGetCell(prhs[2], i);
                if(!mxIsChar(cptr)){
                    mexErrMsgTxt("New: All first argument cell fields must contain only char arrays.");
                }
                getString(cptr, key);
                auto res_ptr = mapobj->find(key);
                if(res_ptr == mapobj->end()){
                    mexErrMsgTxt("lookup: Given key(s) not existent.");
                }
                values.push_back(res_ptr->second);
            }
            
            plhs[0] = mxCreateDoubleMatrix(n, 1, mxREAL);
            double *mxdata = mxGetPr(plhs[0] );
            memcpy(mxdata, values.data(), n*sizeof(double));
        }
        else if(mxIsChar(prhs[2])){
            MEX_GET_STRING("lookup", in, 0)
            auto res_ptr = mapobj->find(in);
            if(res_ptr == mapobj->end()){
                mexErrMsgTxt("lookup: Given key(s) not existent.");
            }
            double out = res_ptr->second;
            MEX_RETURN_DOUBLE(out, 0)            
        }
        else {
            mexErrMsgTxt("lookup: Given input must be a char array or a cell array.");
        }
        
        return;
    }
    
    if(!strcmp("serialize", cmd)) {
        mexErrMsgTxt("serialize: Not yet implemented.");
        return;
    }
    
    if(!strcmp("all_keys", cmd)) {
        MEX_CHECK("all_keys", 0, 1, -1)
        std::vector<std::string> allkeys;
        for(t_map::iterator it = mapobj->begin(); it != mapobj->end(); ++it) {
            allkeys.push_back(it->first);
        }
        if(nlhs == 0){
            return;
        }
        
        plhs[0] = mxCreateCellMatrix(1, allkeys.size());
        char* buffer;
        for(int i = 0; i < allkeys.size(); ++i){            
            int l = allkeys[i].size();
            buffer = (char*) mxCalloc(l+1, sizeof(char));
            memcpy(buffer, allkeys[i].data(), sizeof(char)*l); 
            buffer[l] = '\0'; 
            mxSetCell(plhs[0], i, mxCreateString(buffer));
        }
        
        return;
    }
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}




/**
 * IMPORTANT:
 * Requires prhs has at least length 2!!!
 * prhs[0] must contain the key(s) and prhs[1] the value(s)
 * 
 * 
 * 
 */
void getInput(const mxArray** prhs, double** value_ptr, int& n, std::vector<std::string>& keys){
    n = 0;
    keys.clear();
    
    
    if(!mxIsNumeric(prhs[1]) || mxIsComplex(prhs[1])){
        mexErrMsgTxt("New: Second argument must be a double array.");
    }
    n = mxGetNumberOfElements(prhs[1]);
    
    if((n > 1 && !mxIsCell(prhs[0])) || (n == 1 && !mxIsChar(prhs[0]))){
        mexErrMsgTxt("New: First argument must be a cell array or, if only one value is given, a char array.");
    }
    
    if(mxIsCell(prhs[0]) && n != mxGetNumberOfElements(prhs[0])){
        mexErrMsgTxt("New: The two given arrays must have the same length.");
    }
    
    *value_ptr = mxGetPr(prhs[1]);
    std::string key;
    if(n==1 && mxIsChar(prhs[0])){
        getString(prhs[0], key);
        keys.push_back(key);
    }
    else {
        for(int i = 0; i < n; ++i){
            mxArray* cptr = mxGetCell(prhs[0], i);
            if(!mxIsChar(cptr)){
                mexErrMsgTxt("New: All first argument cell fields must contain only char arrays.");
            }
            getString(cptr, key);
            keys.push_back(key);
        }
    }
}
