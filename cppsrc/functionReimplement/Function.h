/**
 * Copyright (C) 2013 Tobias P. Becker
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the  rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * More information at: https://dslib.assembla.com/
 *
 */
#pragma once

#include <iostream>
#include <string>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>

#include "../math/definitions.h"
// #include "../serialization/serialization_helper.h"

namespace ofc {
    namespace functionLib {
        
        
        class Function {
        protected:
            friend class boost::serialization::access;
            size_t _dimX;
            size_t _dimY;
        public:
            Function() : _dimX(0), _dimY(0){}
            Function(size_t dimX, size_t dimY) : _dimX(dimX), _dimY(dimY) {}
            virtual ~Function(){}
            Function(const Function& f) : _dimX(f._dimX), _dimY(f._dimY) {}
            
            virtual bool evaluate(math::Real* result, const math::Real* x, size_t nx) const{
                for(size_t i = 0; i < nx; i++){
                    if(!evaluate(&result[_dimY*i], &x[_dimX*i])) return false;
                }
                return true;
            }
            virtual bool evaluate(math::Real* result, const math::Real* x) const{
                return false;
            }
            
            /**
             * Degree-th derivative of the Function at the given position with respect to the input arguments.
             * Make sure to check the return value, as not all type of functions support this operation
             * The full vector or matrix representing the derivative is saved consecutively for all nx input values
             * in the result array, which must have preallocated at least _dimY*_dimX*nx fields:
             * result = ( x(0)->[_dimY*_dimX] , ... , x(_dimX*(nx-1))->[_dimY*_dimX] )
             * 
             */
            virtual bool evaluateDerivative(math::Real* result, const math::Real* x, size_t nx, size_t degree) const {
                return false;
            }
            
            size_t NX() const { return _dimX;}
            size_t NY() const { return _dimY;}
            
            virtual Function* copy() const {
                return new Function(_dimX, _dimY);
            }
            
            virtual Function* copyWithCroppedOutput() const {
                return new Function(_dimX, _dimY-1);
            }
                        
            
            static void serialize(const Function* f, std::ostream& s);
            
            static bool unserialize( Function** f, std::istream& s);
            
            std::string serialize() const {
                std::ostringstream os;
                Function::serialize(this, os);
                return os.str();
            }
            
            static Function* unserialize(const std::string& s) {
                std::istringstream is(s);
                Function* ptr;
                Function::unserialize(&ptr, is);
                return ptr;
            }
        private:
            
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version) {
                ar & _dimX;
                ar & _dimY;
            }
        };
        
    }
}


BOOST_CLASS_EXPORT_KEY(ofc::functionLib::Function)
