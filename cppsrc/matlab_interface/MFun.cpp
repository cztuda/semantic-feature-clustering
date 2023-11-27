#include "MFun.h"


BOOST_CLASS_EXPORT_IMPLEMENT(ofc::MFun)

using namespace ofc;

MFun::MFun() : Function(0,0) {
    handleptr[0] = 0;
    handleptr[1] = 0;
}


MFun::MFun(mxArray* function_ptr, int input_size, int output_size) : Function(input_size, output_size) {
    if(!mxIsClass(function_ptr, "function_handle")){
        mexErrMsgTxt("First input argument is not a function handle.");
    }
    handleptr[0] = function_ptr;
    handleptr[1] = mxCreateDoubleMatrix(input_size, 1, mxREAL);
    mexMakeArrayPersistent(handleptr[0]);
    mexMakeArrayPersistent(handleptr[1]);
}


MFun::~MFun(){
    if(handleptr[0]) mxDestroyArray(handleptr[0]);
    if(handleptr[1]) mxDestroyArray(handleptr[1]); // added later, does this cause a crash or why did I not implemented this line before?
}


bool MFun::evaluate(ofc::math::Real* result, const ofc::math::Real* x) const {
    // do not override, as this function must not be const
    
    if(!this->handleptr[0]) return false;
    
    mxArray *lhs;
    
    // copy data from VectorND to matlab-vector
    double* mxindata = mxGetPr(handleptr[1]);
    memcpy(mxindata, x, _dimX*sizeof(double));
    
    // execute matlab function
    mexCallMATLAB(1, &lhs, 2, handleptr, "feval");
    
    // retrieve data and store in result
    double* ptr = mxGetPr(lhs);
    int nout = mxGetNumberOfElements(lhs);
    if(_dimY < nout){
        memcpy(result, mxGetPr(lhs), _dimY*sizeof(double));
    }else {
        memcpy(result, mxGetPr(lhs), nout*sizeof(double));
    }
    memcpy(result, ptr, _dimY*sizeof(double));
    return true;
}


MFun* MFun::copy() const {
    MFun* f;
    if(handleptr[0]) {
        mxArray* handle = mxDuplicateArray(handleptr[0]);
        f = new MFun(handle, this->_dimX, this->_dimY);
    }
    else {
        f = new MFun();
        f->_dimX = this->_dimX;
        f->_dimY = this->_dimY;
    }
    return f;
}


void MFun::serialize(const MFun* f, std::ostream& s) {
    boost::archive::polymorphic_text_oarchive  ar(s);
    ar << f;
}


bool MFun::unserialize( MFun** f, std::istream& s){
    MFun* tmp;
    boost::archive::polymorphic_text_iarchive ia(s);
    ia >> tmp;
    *f = tmp;
    return true;
}


std::string MFun::serialize() const {
    std::ostringstream os;
    MFun::serialize(this, os);
    return os.str();
}


MFun* MFun::unserialize(const std::string& s) {
    std::istringstream is(s);
    MFun* ptr;
    MFun::unserialize(&ptr, is);
    return ptr;
}

