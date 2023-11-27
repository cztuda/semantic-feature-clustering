#pragma once

/*
 * Copyright (c) 2012, Oliver Woodford
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the distribution
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 * 
 * This code has been provided by Oliver Woodford on
 * MathWorks File Exchange :
 * https://de.mathworks.com/matlabcentral/fileexchange/38964-example-matlab-class-wrapper-for-a-c++-class.
 * File downloaded on Jan 05, 2018.
 * 
 * The idea as well as an example implementation how to persistently and robustly 
 * keep C++ objects between mex calls is taken from the webpage cited above and 
 * from the corresponding newsgroup thread "Making C++ objects persistent between 
 * mex calls, and robust.":
 * https://groups.google.com/forum/#!msg/comp.soft-sys.matlab/rp1rxs4348A/EIojHOCZ9LcJ
 * Page visited on Jan 04, 2018
 * 
 * 
 */
//#include "mex_directives.h"


#ifndef __CLASS_HANDLE_HPP__
#define __CLASS_HANDLE_HPP__
#include "mex.h"
#include <stdint.h>
#include <string>
#include <cstring>
#include <typeinfo>

#define CLASS_HANDLE_SIGNATURE 0xFF00F0A5
template<class base> class class_handle
{
public:
    class_handle(base *ptr) : name_m(typeid(base).name()), ptr_m(ptr) { signature_m = CLASS_HANDLE_SIGNATURE; }
    ~class_handle() { signature_m = 0; delete ptr_m; }
    bool isValid() { 
      bool valid = ((signature_m == CLASS_HANDLE_SIGNATURE) && !strcmp(name_m.c_str(), typeid(base).name()));
      if(!valid)
      {
	mexPrintf("\nClass handle validity check failed:\n");
	mexPrintf(" signature_m=%i (expected signature: %i)\nname=%s (expected name=%s)\n", signature_m, CLASS_HANDLE_SIGNATURE, name_m.c_str(), typeid(base).name());
	mexEvalString("drawnow;");
      }
      return valid; 
    }
    base *ptr() { return ptr_m; }

private:
    uint32_t signature_m;
    std::string name_m;
    base *ptr_m;
};

template<class base> inline mxArray *convertPtr2Mat(base *ptr)
{
    mexLock();
    mxArray *out = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    *((uint64_t *)mxGetData(out)) = reinterpret_cast<uint64_t>(new class_handle<base>(ptr));
    return out;
}

template<class base> inline class_handle<base> *convertMat2HandlePtr(const mxArray *in)
{
    if (mxGetNumberOfElements(in) != 1 || mxGetClassID(in) != mxUINT64_CLASS || mxIsComplex(in))
        mexErrMsgTxt("Input must be a real uint64 scalar.");
    class_handle<base> *ptr = reinterpret_cast<class_handle<base> *>(*((uint64_t *)mxGetData(in)));
    if (ptr != nullptr && !ptr->isValid()){
        mexErrMsgTxt("Handle not valid.");
    }
    return ptr;
}

template<class base> inline bool convertMat2HandlePtr(class_handle<base> ** retval, const mxArray *in)
{
  if (mxGetNumberOfElements(in) != 1 || mxGetClassID(in) != mxUINT64_CLASS || mxIsComplex(in))
        mexErrMsgTxt("Input must be a real uint64 scalar.");
    try {
      class_handle<base> *ptr = reinterpret_cast<class_handle<base> *>(*((uint64_t *)mxGetData(in)));
      if (ptr->isValid()){
	*retval = ptr;
	return true;
      }
    }
    catch(...){
    }
    return false;
}

template<class base> inline base* tryConvertMat2Ptr(const mxArray* in)
{
  class_handle<base>* hptr;
  bool succ = convertMat2HandlePtr<base>(&hptr, in);
  if(succ){
    return hptr->ptr();
  }
  else {
    return 0;
  }
}

template<class base> inline base *convertMat2Ptr(const mxArray *in)
{
    return convertMat2HandlePtr<base>(in)->ptr();
}

template<class base> inline void destroyObject(const mxArray *in)
{
    delete convertMat2HandlePtr<base>(in);
    mexUnlock();
}

#endif // __CLASS_HANDLE_HPP__
