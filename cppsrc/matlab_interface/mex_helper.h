#pragma once

#include "mex.h"
#include "class_handle.hpp"

#include <cstdio>
#include <vector>
#include <type_traits>
#include <Eigen/Core>
#include <functional>

struct MEX_PTR{
    int nlhs;
    int nrhs; 
    mxArray** plhs;
    const mxArray** prhs;
    MEX_PTR(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
        : nlhs(nlhs), nrhs(nrhs), plhs(plhs), prhs(prhs) {};
    MEX_PTR() : nlhs(0), nrhs(0), plhs(nullptr), prhs(nullptr){};
};
MEX_PTR mexdata;


namespace mex_internal {
    
    template<typename T>
    inline void put_double(const T& value, mxArray** dest){
        if(std::is_same<T, double>::value){
            *dest = mxCreateDoubleScalar(value);
        }
        else {
            *dest = mxCreateDoubleScalar(static_cast<double>(value));
        }
    }
    inline void put_string(const std::string& value, mxArray** dest){
        int l = value.size();
        char* buffer = (char*) mxCalloc(l+1, sizeof(char));
        memcpy(buffer, value.data(), sizeof(char)*l);
        buffer[l] = '\0';
        *dest = mxCreateString(buffer);
    }
    
    
    template<typename T>
    inline void put_double_container(const T& container, mxArray** dest){
        int n = container.size();
        *dest = mxCreateDoubleMatrix( n, 1, mxREAL );
        double* mxdata = mxGetPr(*dest);
        int i = 0;
        for(auto it = container.begin(); it < container.end(); ++it){
            mxdata[i++] = *it;
        }
    }
    template<typename T>
    inline void put_double_container2(const T& container2, mxArray** dest){
        int nOuter = container2.size();
        *dest = mxCreateCellMatrix( nOuter, 1 );
        int i = 0;
        for(auto it = container2.begin(); it < container2.end(); ++it){
            mxArray* tmp = nullptr;
            put_double_container(*it, &tmp);
            mxSetCell(*dest, i++, tmp);
        }
    }
    template<typename T>
    inline void put_string_container(const T& container, mxArray** dest){
        int n = container.size();
        *dest = mxCreateCellMatrix( n, 1 );
        int i = 0;
        for(auto it = container.begin(); it < container.end(); ++it){
            mxArray* tmp = nullptr;
            put_string(*it, &tmp);
            mxSetCell(*dest, i++, tmp);
        }
    }
    
    
    template<typename T>
    inline void put_EigenMatrix(mxArray** A, const Eigen::MatrixX<T>& mat, bool createNew=true){
        int nrow = mat.rows();
        int ncol = mat.cols();
        if(createNew){
            *A = mxCreateDoubleMatrix(nrow, ncol, mxREAL);
        }
        double* mxdata = mxGetPr(*A);
        if(std::is_same<T, double>::value){
            for(int i = 0; i < ncol; ++i){
                for(int j = 0; j < nrow; ++j){
                    mxdata[nrow*i+j] = mat(j, i);
                }
            }
        }
        else {
            for(int i = 0; i < ncol; ++i){
                for(int j = 0; j < nrow; ++j){
                    mxdata[nrow*i+j] = static_cast<double>(mat(j, i));
                }
            }
        }
    }
    template<typename T>
    inline void put_EigenVector(mxArray** A, const Eigen::VectorX<T>& mat, bool createNew=true){
        int nrow = mat.size();
        
        if(createNew){
            *A = mxCreateDoubleMatrix(nrow, 1, mxREAL);
        }
        double* mxdata = mxGetPr(*A);
        if(std::is_same<T, double>::value){
            for(int j = 0; j < nrow; ++j){
                mxdata[j] = mat(j);
            }
        }
        else {
            for(int j = 0; j < nrow; ++j){
                mxdata[j] = static_cast<double>(mat(j));
            }
        }
    }
    
    template<typename T>
    inline void put_obj_ptr(mxArray** A, T* obj){
        // automatically creates a new object!
        *A = convertPtr2Mat<T>(obj);
    }
}


template<typename... Args>
inline void mex_error(const std::string& msg, Args... args){
    const size_t len = 1024;
    char tmp[len];
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wformat"
    std::snprintf(tmp, len, msg.c_str(), args...);
    #pragma GCC diagnostic pop    
    mexErrMsgTxt(tmp);
}

template<int offset=2>
inline void mex_check(const std::string& name, MEX_PTR& data, int n_in_req, int n_out_max, int n_out_min=-1){
    if(data.nrhs < n_in_req+offset){
        mex_error("%s: Additional argument(s) required.", name.c_str());
    }
    if(data.nlhs > n_out_max){
        mex_error("%s: Too many output arguments.", name.c_str());
    }
    if(data.nlhs < n_out_min && n_out_min >= 0){
        mex_error("%s: Expected at least %i output arguments.", name.c_str(), n_out_min);
    }
}

inline void mex_put_empty(MEX_PTR& data, int index){
  if(data.nlhs > index) {
    data.plhs[index] = mxCreateDoubleMatrix(0, 0, mxREAL);
  }  
}
  

template<typename T>
inline void mex_put_double(MEX_PTR& data, const T& value, unsigned int index){
    if(data.nlhs > index){
        mex_internal::put_double(value, &data.plhs[index]);
    }
}

inline void mex_put_string(MEX_PTR& data, const std::string& str, int index){
    if(data.nlhs > index){
        int l = str.size();
        char* buffer = (char*) mxCalloc(l+1, sizeof(char));
        memcpy(buffer, str.data(), sizeof(char)*l);
        buffer[l] = '\0';
        data.plhs[index] = mxCreateString(buffer);
    }
}

template<typename T>    
inline void mex_put_double_container(MEX_PTR& data, const int index, const T& container){
    if(index < data.nlhs){
        mex_internal::put_double_container(container, &data.plhs[index]);
    }
}

template<unsigned int offset=2, typename T>
void mex_put_double_container2(MEX_PTR& data, int index, const T& container2){
    /*
     * convert e.g. std::vector<std::vector<double>> to cell of arrays in Matlab
     */
    if(index < data.nlhs){
        mex_internal::put_double_container2(container2, &data.plhs[index]);
    }
}

template<typename T>
inline void mex_put_EigenMatrix(MEX_PTR& data, const int index, const Eigen::MatrixX<T>& mat, bool createNew=true){
    if(index < data.nlhs){
        mex_internal::put_EigenMatrix(&data.plhs[index], mat, createNew);
    }
}
template<typename T>
inline void mex_put_EigenVector(MEX_PTR& data, const int index, const Eigen::VectorX<T>& mat, bool createNew=true){
    if(index < data.nlhs){
        mex_internal::put_EigenVector(&data.plhs[index], mat, createNew);
    }
}

template<unsigned int offset=2, typename T>
inline void mex_pull_double(MEX_PTR& data, const std::string& name, T& var, int index){
    if(!mxIsDouble(data.prhs[index+offset])){
        mex_error("%s: Input %i must be a double.", name.c_str(), index+1);
    }
    double d = *mxGetDoubles(data.prhs[index+offset]);
    if(std::is_same<T, double>::value){
        var = d;
    }
    else {
        var = static_cast<T>(d);
    }
}

template<unsigned int offset=2>
inline void mex_pull_string(MEX_PTR& data, const std::string& name, std::string& var, int index){
    if(!mxIsChar(data.prhs[index+offset])){
        mex_error("%s: Input %i must be a string.", name.c_str(), index+1);
    }
    mwSize buflen = mxGetN(data.prhs[index+offset])*sizeof(mxChar)+1;
    char* c_array = new char[buflen];
    mxGetString(data.prhs[index+offset],c_array,buflen);
    var = std::string(c_array);
    delete [] c_array;
    
}

template<unsigned int offset=2, typename T>
inline void mex_pull_stdvector(MEX_PTR& data, const std::string& name, std::vector<T>& var, int index){
    auto ptr = data.prhs[index+offset];
    if(!mxIsDouble(ptr))
        mex_error("%s: Input %i must be a double value.", name.c_str(), index+1);
    int len = mxGetNumberOfElements(ptr);
    double* p = mxGetPr(ptr);
    if(std::is_same<T, double>::value){
        var = std::vector<T>(p, p+len);
    }
    else {
        var.resize(len);
        for(int i = 0; i < len; ++i){
            var[i] = static_cast<T>(p[i]);
        }
    }
}

template<unsigned int offset=2, typename T=double>
inline void mex_pull_eigenVector(MEX_PTR& data, const std::string& name, Eigen::VectorX<T>& var, int index){
    auto ptr = data.prhs[index+offset];
    if(!mxIsDouble(ptr))
        mex_error("%s: Input %i must be a double value.", name.c_str(), index+1);
    int len = mxGetNumberOfElements(ptr);
    double* p = mxGetPr(ptr);
    var.resize(len);
    if(std::is_same<T, double>::value){
        Eigen::Map<Eigen::MatrixXd> tmp(p, len, 1);
        var = tmp;
    }
    else {
        for(int i = 0; i < len; ++i){
            var(i) = static_cast<T>(p[i]);
        }
    }
}

template<unsigned int offset=2, typename T=double>
inline void mex_pull_eigenMatrix(MEX_PTR& data, const std::string& name, Eigen::MatrixX<T>& var, int index){
    auto ptr = data.prhs[index+offset];
    if(!mxIsDouble(ptr))
        mex_error("%s: Input %i must be a double value.", name.c_str(), index+1);
    int m = mxGetM(ptr);
    int n = mxGetN(ptr);
    double* p = mxGetPr(ptr);
    var.resize(m, n);
    if(std::is_same<T, double>::value){
        Eigen::Map<Eigen::MatrixXd> tmp(p, m, n);
        var = tmp;
    }
    else {
        for(int i = 0; i < n; ++i){
            for(int j = 0; j < m; ++j){
                var(j, i) = static_cast<T>(p[i*m+j]);
            }
        }
    }
}

template<typename T, unsigned int offset=2>
inline T* mex_pull_object(MEX_PTR& data, const std::string& name, int index){
    T* obj = convertMat2Ptr<T>(data.prhs[index+offset]);
    if(!obj)
        mex_error("%s: Cannot load object at input %i.", name.c_str(), index+1);
    return obj;
}


// Fully shaped blocks
template<typename T>
inline void mex_block_get_double(MEX_PTR& data, const std::string& name, std::function<T()> fun){
    mex_check(name, data, 0, 1);
    if(std::is_same<T, double>::value){
        mex_put_double(data, fun(), 0);
    }
    else {
        mex_put_double(data, static_cast<double>(fun()), 0);
    }    
}
template<typename T>
inline void mex_block_set_double(MEX_PTR& data, const std::string& name, std::function<void(T)> fun){
    mex_check(name, data, 1, 0);
    double val;
    mex_pull_double(data, name, val, 0);
    if(std::is_same<T, double>::value){
        fun(val);
    }
    else {
        fun(static_cast<T>(val));
    }    
}

inline void mex_block_get_string(MEX_PTR& data, const std::string& name, std::function<std::string()> fun){
    mex_check(name, data, 0, 1);
    mex_put_string(data, fun(), 0); 
}
inline void mex_block_set_string(MEX_PTR& data, const std::string& name, std::function<void(std::string&)> fun){
    mex_check(name, data, 1, 0);
    std::string val;
    mex_pull_string(data, name, val, 0);
    fun(val);
}


// Dealing with structures
inline void mex_put_struct_create(MEX_PTR& data, const std::string& name, std::vector<std::string>& fieldnames, int index){
    const int n = fieldnames.size();
//     char** nameptr = new char*[n];
    std::vector<const char*> nameptr(fieldnames.size());
    for(int i = 0; i < n; ++i){
        nameptr[i] = fieldnames[i].c_str();
    }
    data.plhs[index] = mxCreateStructMatrix(1, 1, n, nameptr.data());
}
inline void mex_put_struct_create(MEX_PTR& data, const std::string& name, const char** fieldnames, int n, int index){
    data.plhs[index] = mxCreateStructMatrix(1, 1, n, fieldnames);
}
inline void mex_put_struct_create(mxArray** data, const std::string& name, std::vector<std::string>& fieldnames){
    const int n = fieldnames.size();
//     char** nameptr = new char*[n];
    std::vector<const char*> nameptr(fieldnames.size());
    for(int i = 0; i < n; ++i){
        nameptr[i] = fieldnames[i].c_str();
    }
    *data = mxCreateStructMatrix(1, 1, n, nameptr.data());
}
inline void mex_put_struct_create(mxArray** data, const std::string& name, const char** fieldnames, int n){
    *data = mxCreateStructMatrix(1, 1, n, fieldnames);
}


template<typename T=double>
inline void mex_put_struct_field_double(mxArray* data, const std::string& name, const std::string& fieldname, const T& value){
    mxArray* tmp = nullptr;
    mex_internal::put_double(value, &tmp);
    mxSetField(data, 0, fieldname.c_str(), tmp);
}
template<typename T=double>
inline void mex_put_struct_field_double(MEX_PTR& data, const std::string& name, int index, const std::string& fieldname, const T& value){
    mex_put_struct_field_double(data.plhs[index], name, fieldname, value);
}


inline void mex_put_struct_field_string(mxArray* data, const std::string& name, const std::string& fieldname, const std::string& value){
    mxArray* tmp = nullptr;
    mex_internal::put_string(value, &tmp);
    mxSetField(data, 0, fieldname.c_str(), tmp);
}
inline void mex_put_struct_field_string(MEX_PTR& data, const std::string& name, int index, const std::string& fieldname, const std::string& value){
    mex_put_struct_field_string(data.plhs[index], name, fieldname, value);
}


template<typename T>
inline void mex_put_struct_field_container(mxArray* data, const std::string& name, const std::string& fieldname, const T& container){
    mxArray* tmp = nullptr;
    mex_internal::put_double_container(container, &tmp);
    mxSetField(data, 0, fieldname.c_str(), tmp);
}
template<typename T>
inline void mex_put_struct_field_container(MEX_PTR& data, const std::string& name, int index, const std::string& fieldname, const T& container){
    mex_put_struct_field_container(data.plhs[index], name, fieldname, container);
}

template<typename T>
inline void mex_put_struct_field_string_container(mxArray* data, const std::string& name, const std::string& fieldname, const T& container){
    mxArray* tmp = nullptr;
    mex_internal::put_string_container(container, &tmp);
    mxSetField(data, 0, fieldname.c_str(), tmp);
}
template<typename T>
inline void mex_put_struct_field_string_container(MEX_PTR& data, const std::string& name, int index, const std::string& fieldname, const T& container){
    mex_put_struct_field_string_container(data.plhs[index], name, fieldname, container);
}

template<typename T>
inline void mex_put_struct_field_eigenmatrix(mxArray* data, const std::string& name, const std::string& fieldname, const Eigen::MatrixX<T>& mat){
    mxArray* tmp = nullptr;
    mex_internal::put_EigenMatrix(&tmp, mat);
    mxSetField(data, 0, fieldname.c_str(), tmp);
}
template<typename T>
inline void mex_put_struct_field_eigenmatrix(MEX_PTR& data, const std::string& name, int index, const std::string& fieldname, const Eigen::MatrixX<T>& mat){
    mex_put_struct_field_eigenmatrix(data.plhs[index], name, fieldname, mat);
}


template<typename T>
void mex_put_struct_field_container2(mxArray* data, const std::string& name, const std::string& fieldname, const T& container2){
    mxArray* tmp = nullptr;
    mex_internal::put_double_container2(container2, &tmp);
    mxSetField(data, 0, fieldname.c_str(), tmp);
}
template<typename T>
void mex_put_struct_field_container2(MEX_PTR& data, const std::string& name, int index, const std::string& fieldname, const T& container2){
    mex_put_struct_field_container2(data.plhs[index], name, fieldname, container2);
}


template<typename T>
void mex_put_struct_field_obj(mxArray* data, const std::string& name, const std::string& fieldname, T* obj){
    mxArray* tmp = nullptr;
    mex_internal::put_obj_ptr<T>(&tmp, obj);
    mxSetField(data, 0, fieldname.c_str(), tmp);
}
template<typename T>
void mex_put_struct_field_obj(MEX_PTR& data, const std::string& name, int index, const std::string& fieldname, T* obj){
    mex_put_struct_field_obj(data.plhs[index], name, fieldname, obj);
}


void mex_put_struct_field_matobj(mxArray* data, const std::string& name, const std::string& fieldname, mxArray* matobj){
    mxSetField(data, 0, fieldname.c_str(), matobj);
}
void mex_put_struct_field_matobj(MEX_PTR& data, const std::string& name, int index, const std::string& fieldname, mxArray* matobj){
    mex_put_struct_field_matobj(data.plhs[index], name, fieldname, matobj);
}

void mex_put_struct_field_empty(mxArray* data, const std::string& name, const std::string& fieldname){
    mxSetField(data, 0, fieldname.c_str(), mxCreateDoubleMatrix(0, 0, mxREAL));
}
void mex_put_struct_field_empty(MEX_PTR& data, const std::string& name, int index, const std::string& fieldname){
    mex_put_struct_field_matobj(data.plhs[index], name, fieldname, mxCreateDoubleMatrix(0, 0, mxREAL));
}
