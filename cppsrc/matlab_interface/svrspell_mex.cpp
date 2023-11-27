#include "mex.h"

#include <vector>
#include <cmath>

#include "mex_directives.h"
#include "../util/elzinga_grid.h"
#include "../util/onCleanup.h"
#include <iostream>
#include <cstdlib>


inline mxArray* createHandleDeepCopy(const mxArray* byteStreamObj){
    mxArray *inputs[1], *outputs[1];
    
    inputs[0] = mxDuplicateArray(byteStreamObj);
    
    ofc::onCleanup cleanInput([inputs](){
        mxDestroyArray(inputs[0]);
    });
    
    int status = mexCallMATLAB(1, outputs, 1, inputs, "getArrayFromByteStream");
    if(!status)
        return outputs[0];
    else
        return nullptr;
}

mxArray* ptr_to_handle_lenW;
mxArray* ptr_to_handle_gapW;

std::vector<double> times1;
std::vector<double> times2;

int nx, ny;

double lenW_fun(const svr_num& k, const double& mue_k){
    mxArray *inputs[3], *outputs[1];
    
    inputs[0] = ptr_to_handle_lenW;
    inputs[1] = mxCreateDoubleScalar(k);
    inputs[2] = mxCreateDoubleScalar(mue_k);
    
    ofc::onCleanup cleanInput([inputs](){
        mxDestroyArray(inputs[1]);
        mxDestroyArray(inputs[2]);
    });
    
    int status = mexCallMATLAB(1, outputs, 3, inputs, "feval");
    
    double retval = -1;
    if(!status){
        retval = *mxGetDoubles(outputs[0]);
    }
    return retval;
}

double gapW_fun(const svr_num& in1, const svr_num& in2, const svr_num& in3, const svr_num& in4){
    mxArray *inputs[5], *outputs[1];
    
    inputs[0] = ptr_to_handle_gapW;
    inputs[1] = mxCreateDoubleScalar(in1);
    inputs[2] = mxCreateDoubleScalar(in2);
    inputs[3] = mxCreateDoubleScalar(in3);
    inputs[4] = mxCreateDoubleScalar(in4);
    
    ofc::onCleanup cleanInput([inputs](){
        mxDestroyArray(inputs[1]);
        mxDestroyArray(inputs[2]);
        mxDestroyArray(inputs[3]);
        mxDestroyArray(inputs[4]);
    });
    
    int status = mexCallMATLAB(1, outputs, 5, inputs, "feval");
    
    double retval = -1;
    if(!status){
        retval = *mxGetDoubles(outputs[0]);
    }
    return retval;
}


// inline double myGapW_Fun_helper(const std::vector<double>& a, const std::vector<double>& b, const svr_num in1, const svr_num in2, const svr_num in3, const svr_num in4){
//     return (a[in3]-a[in1])*(b[in4]-b[in2]);
// //     return static_cast<double>(rand())/RAND_MAX;
// }


inline double myGapW_Fun_helper(const std::vector<double>& a, const std::vector<double>& b, const svr_num& in1, const svr_num& in2, const svr_num& in3, const svr_num& in4){
//     std::cout << a[in3] << " " << a[in1] << " " << b[in4] << " " << b[in2] << std::endl;
    auto val = 1 - abs((a[in3]-a[in1])-(b[in4]-b[in2]));
//     std::cout << val << std::endl;
    return val;
}

double myGapW_fun(const svr_num& in1, const svr_num& in2, const svr_num& in3, const svr_num& in4){
    double val = myGapW_Fun_helper(times1, times2, in1, in2, in3, in4);
    // auto val = (times1[in3]-times1[in1])*(times2[in4]-times2[in2]);
    return std::sqrt(val);
}
double myGapW_fun_yy(const svr_num& in1, const svr_num& in2, const svr_num& in3, const svr_num& in4){
    double val = myGapW_Fun_helper(times2, times2, in1, in2, in3, in4);
    //auto val = (times2[in3]-times2[in1])*(times2[in4]-times2[in2]);
    return std::sqrt(val);
}
double myGapW_fun_xx(const svr_num& in1, const svr_num& in2, const svr_num& in3, const svr_num& in4){
    double val = myGapW_Fun_helper(times1, times1, in1, in2, in3, in4);
    //auto val = (times1[in3]-times1[in1])*(times1[in4]-times1[in2]);
    return std::sqrt(val);
}

/*
 * softMatchingMatrix
 * charWeights
 * lengthWFun
 * gapWFun 
 * tx
 * ty
 */
SVRspellOptions getInputArguments(int nrhs, const mxArray *prhs[]){
    Eigen::MatrixXd softMatchingMatrix;
    Eigen::VectorXd weights;
    lwptr lengthWeighting=nullptr;
    gwptr gapWeighting=nullptr;
    gwptr gapWeighting_xx=nullptr;
    gwptr gapWeighting_yy=nullptr;
    std::vector<svr_val> tx; 
    std::vector<svr_val> ty;
    std::vector<svr_val> vx; 
    std::vector<svr_val> vy;
    
    ///
    if(nrhs > 2 && !mxIsEmpty(prhs[2])){
        mxArrayToEigenMatrix(prhs[2], softMatchingMatrix);
    }
    else {
        softMatchingMatrix = Eigen::MatrixXd(0,0);
    }
    
    ///
    if(nrhs > 3 && !mxIsEmpty(prhs[3])){
        mxArrayToEigenVector(prhs[3], weights);
    }
    else {
        weights = Eigen::VectorXd(0);
    }
    
    ///
    if(nrhs > 4 && !mxIsEmpty(prhs[4])){
        ptr_to_handle_lenW = createHandleDeepCopy(prhs[4]);
        lengthWeighting = &lenW_fun;     
    }
    
    ///
    if(nrhs > 5 && !mxIsEmpty(prhs[5])){
        if(mxIsStruct(prhs[5])){
            mxArray* tmp = mxGetField(prhs[5], 0, "x");
            int len; 
            double* p;
            if(tmp != nullptr){
                len = mxGetNumberOfElements(tmp);
                p = mxGetPr(tmp);
                times1 = std::vector<double>(p, p+len);
                if(times1.size() < nx) {
                    times1.resize(nx);
                }
            }
            else {
                times1.resize(nx);
            }
            tmp = mxGetField(prhs[5], 0, "y");
            if(tmp != nullptr){
                len = mxGetNumberOfElements(tmp);
                p = mxGetPr(tmp);
                times2 = std::vector<double>(p, p+len);
                if(times2.size() < ny) {
                    times2.resize(ny);
                }
            }
            else {
                times1.resize(ny);
            }
            gapWeighting = &myGapW_fun;
            gapWeighting_xx = &myGapW_fun_xx;
            gapWeighting_yy = &myGapW_fun_yy;
        }
        else {
            ptr_to_handle_gapW = createHandleDeepCopy(prhs[5]);
            gapWeighting = &gapW_fun;
        }        
    }
    
    ///
    if(nrhs > 6 && !mxIsEmpty(prhs[6])){
        int len = mxGetNumberOfElements(prhs[6]);
        double* p = mxGetPr(prhs[6]);
        tx = std::vector<double>(p, p+len);
    }
    else {
        tx=std::vector<double>();
    }
    
    ///
    if(nrhs > 7 && !mxIsEmpty(prhs[7])){
        int len = mxGetNumberOfElements(prhs[7]);
        double* p = mxGetPr(prhs[7]);
        ty = std::vector<double>(p, p+len);
    }
    else {
        ty=std::vector<double>();
    }
    
    ///
    if(nrhs > 8 && !mxIsEmpty(prhs[8])){
        int len = mxGetNumberOfElements(prhs[8]);
        double* p = mxGetPr(prhs[8]);
        vx = std::vector<double>(p, p+len);
    }
    else {
        vx=std::vector<double>();
    }
    
    ///
    if(nrhs > 9 && !mxIsEmpty(prhs[9])){
        int len = mxGetNumberOfElements(prhs[9]);
        double* p = mxGetPr(prhs[9]);
        vy = std::vector<double>(p, p+len);
    }
    else {
        vy=std::vector<double>();
    }
    
    return SVRspellOptions(softMatchingMatrix, weights, lengthWeighting, gapWeighting, gapWeighting_xx, gapWeighting_yy, tx, ty, vx, vy);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    // double distance(const std::vector<sym>& x, const std::vector<sym>& y, double& xy, double& xx, double& yy)
    
    MEX_CHECK(svrspell, 2-2, 4, -1)  
    if (nlhs == 0) {return;}
    nx = mxGetNumberOfElements(prhs[0]);
    ny = mxGetNumberOfElements(prhs[1]);
    
    std::vector<svr_sym> X(nx), Y(ny);
    double *x_ptr, *y_ptr;
    
    MEX_GET_DOUBLE_PTR(svrspell, x_ptr, nx, -2)
    MEX_GET_DOUBLE_PTR(svrspell, y_ptr, ny, -1)
    
    for(size_t i = 0; i < nx; ++i){
        X[i] = static_cast<svr_sym>(x_ptr[i]);
    }
    for(size_t i = 0; i < ny; ++i){
        Y[i] = static_cast<svr_sym>(y_ptr[i]);
    }
    
    SVRspellOptions options = getInputArguments(nrhs, prhs);
    if(options.gapWeighting() != nullptr){
        
    }
    
    double d, xx, yy, xy;
    d = distance(X, Y, options, xy, xx, yy);
    switch(nlhs){
        case 4:
            MEX_RETURN_DOUBLE(yy, 3)
        case 3:
            MEX_RETURN_DOUBLE(xx, 2)
        case 2:
            MEX_RETURN_DOUBLE(xy, 1)
        case 1:
            MEX_RETURN_DOUBLE(d, 0)
    }
    
    if(ptr_to_handle_lenW != nullptr){
        mxDestroyArray(ptr_to_handle_lenW);
        ptr_to_handle_lenW = nullptr;
    }
    if(ptr_to_handle_gapW != nullptr){
        mxDestroyArray(ptr_to_handle_gapW);
        ptr_to_handle_gapW = nullptr;
    }
    times1.resize(0);
    times2.resize(0);
}
