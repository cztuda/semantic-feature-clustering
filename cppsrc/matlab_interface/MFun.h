#pragma once

#include "mex.h"

#include <iostream>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/array_wrapper.hpp>
#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>

#include "../functionReimplement/Function.h"


// MX_API_VER has unfortunately not changed between R2013b and R2014a,
// so we use the new MATRIX_DLL_EXPORT_SYM as an ugly hack instead
//#if defined(__cplusplus) && defined(MATRIX_DLL_EXPORT_SYM)
#define EXTERN_C extern
namespace matrix{ namespace detail{ namespace noninlined{ namespace mx_array_api{
    //#endif
    
    EXTERN_C mxArray* mxSerialize(mxArray const *);
    EXTERN_C mxArray* mxDeserialize(const void *, size_t);
    // and so on, for any other MEX C functions that migrated to C++ in R2014a
    
    //#if defined(__cplusplus) && defined(MATRIX_DLL_EXPORT_SYM)
}}}}
using namespace matrix::detail::noninlined::mx_array_api;
//#endif

namespace ofc {

class MFun : public ofc::functionLib::Function {
private:
    // handleptr[0]: the Matlab function handle
    // handleptr[1]: argument x for the function handle, a Matlab array
    // Use of mutable is an abuse! In fact, only handleptr[1] should be mutable, but this is not possible. So make sure to not change handleptr[0] in const-functions.
    mutable mxArray* handleptr[2];
public:
    MFun();
    MFun(mxArray* function_ptr, int input_size, int output_size);
    ~MFun();
    
    virtual bool evaluate(ofc::math::Real* result, const ofc::math::Real* x) const override;
    
    virtual MFun* copy() const override;
    
    static void serialize(const MFun* f, std::ostream& s);
    static bool unserialize( MFun** f, std::istream& s);
    std::string serialize() const;
    static MFun* unserialize(const std::string& s);
    
private:
    friend class boost::serialization::access;
    
    template<class Archive>
    void save(Archive & ar, const unsigned int version) const
    {
        ar & boost::serialization::base_object<ofc::functionLib::Function>(*this);
        
        uint8_t* data;
        size_t len;
        if(handleptr[0]) {
            mxArray* dataArray = (mxArray*) mxSerialize(this->handleptr[0]);
            data = (uint8_t*) mxGetUint8s(dataArray);
            len = mxGetNumberOfElements(dataArray);
        }
        else {
            len = 0;
            data = 0;
        }
        
        // save len, data and base class attributes
        ar & len;
        ar & boost::serialization::make_array<uint8_t>(data, len);
    }
    
    template<class Archive>
    void load(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<ofc::functionLib::Function>(*this);
        
        size_t len;
        std::vector<uint8_t> data;
        
        ar & len;
        
        data.reserve(len);
        ar & boost::serialization::make_array<uint8_t>(data.data(), len);
        
        if(handleptr[0]) { 
            mxDestroyArray(handleptr[0]); 
            handleptr[0] = 0;
        }
        if(handleptr[1]) { 
            mxDestroyArray(handleptr[1]); 
            handleptr[1] = 0;
        }
        
        if(len > 0) {
            handleptr[0] = (mxArray *) mxDeserialize(data.data(), len);
            handleptr[1] = mxCreateDoubleMatrix(this->_dimX, 1, mxREAL);
            mexMakeArrayPersistent(handleptr[0]);
            mexMakeArrayPersistent(handleptr[1]);
        }
    }
    
    BOOST_SERIALIZATION_SPLIT_MEMBER()
};

}

BOOST_CLASS_EXPORT_KEY(ofc::MFun)

