#include "SingleVariableMFun.h"


BOOST_CLASS_EXPORT_IMPLEMENT(ofc::SingleVariableMFun)

using namespace ofc;

SingleVariableMFun::SingleVariableMFun() : ofc::functionLib::SingleVariableFunction(0) 
{
    handleptr[0] = 0;
    handleptr[1] = 0;
}


SingleVariableMFun::SingleVariableMFun(mxArray* function_ptr, int output_size) : SingleVariableFunction(output_size){
    if(!mxIsClass(function_ptr, "function_handle")){
        mexErrMsgTxt("First input argument is not a function handle.");
    }
    handleptr[0] = function_ptr;
    handleptr[1] = mxCreateDoubleScalar(0);
    mexMakeArrayPersistent(handleptr[0]);
    mexMakeArrayPersistent(handleptr[1]);
}


SingleVariableMFun::~SingleVariableMFun(){
    if(handleptr[0]) mxDestroyArray(handleptr[0]);
    if(handleptr[1]) mxDestroyArray(handleptr[1]);
}


bool SingleVariableMFun::evaluate(ofc::math::Real * result, const ofc::math::Real* x) const { 
    // do not override, as this function must not be const
    
    if(!this->handleptr[0]) return false;
    
    mxArray *lhs;
    
    // copy data from VectorND to matlab-vector
    double* mxindata = mxGetPr(handleptr[1]);
    mxindata[0] = x[0];
    
    // execute matlab function
    mexCallMATLAB(1, &lhs, 2, handleptr, "feval");
    
    // retrieve data and store in result
    int nout = mxGetNumberOfElements(lhs);
    if(_dimY < nout){
        memcpy(result, mxGetPr(lhs), _dimY*sizeof(double));
    }else {
        memcpy(result, mxGetPr(lhs), nout*sizeof(double));
    }
    return true;
}


SingleVariableMFun* SingleVariableMFun::copy() const {
    SingleVariableMFun* f;
    if(handleptr[0]) {
        mxArray* handle = mxDuplicateArray(handleptr[0]);
        f = new SingleVariableMFun(handle, this->_dimY);
    }
    else {
        f = new SingleVariableMFun();
        f->_dimX = this->_dimX;
        f->_dimY = this->_dimY;
    }
    return f;
}


void SingleVariableMFun::serialize(const SingleVariableMFun* f, std::ostream& s) {
    boost::archive::polymorphic_text_oarchive  ar(s);
    ar << f;
}


bool SingleVariableMFun::unserialize( SingleVariableMFun** f, std::istream& s){
    SingleVariableMFun* tmp;
    boost::archive::polymorphic_text_iarchive ia(s);
    ia >> tmp;
    *f = tmp;
    return true;
}


std::string SingleVariableMFun::serialize() const {
    std::ostringstream os;
    SingleVariableMFun::serialize(this, os);
    return os.str();
}


SingleVariableMFun* SingleVariableMFun::unserialize(const std::string& s) {
    std::istringstream is(s);
    SingleVariableMFun* ptr;
    SingleVariableMFun::unserialize(&ptr, is);
    return ptr;
}

