#pragma once

#include "../serialization/serialization_helper.h"
#include <boost/serialization/base_object.hpp>

#include <limits>
#include <cmath>

#include "Function.h"
#include "Interval.h"
#include "../math/definitions.h"

namespace ofc {
    namespace functionLib {
        
        
        class SingleVariableFunction : public Function {
        protected:
            double t0;
            double tf;
        public:
            SingleVariableFunction() : Function(), t0(-std::numeric_limits<double>::max()), tf(std::numeric_limits<double>::max()) {};
            SingleVariableFunction(size_t dimY) : Function(1, dimY), t0(-std::numeric_limits<double>::max()), tf(std::numeric_limits<double>::max()) {};
            SingleVariableFunction(size_t dimY, double t0, double tf);
            SingleVariableFunction(const SingleVariableFunction& svf);
            
            virtual ~SingleVariableFunction();
            
            double getT0() const;
            double getTf() const;
            
            virtual bool evaluate(math::Real* result, const math::Real* x) const override;
            
            virtual bool evaluateDerivative(math::Real* result, const math::Real* x, size_t nx, size_t degree) const override;
                
            virtual SingleVariableFunction* copy() const override;
            
            virtual SingleVariableFunction* copyWithCroppedOutput() const override;
            virtual SingleVariableFunction* copyWithDomainMapping(double t0=NAN, double tf=NAN) const;
            virtual SingleVariableFunction* copyWithDomainCut(double t0=NAN, double tf=NAN) const;
            virtual SingleVariableFunction* copyWithDomainShift(const double offset) const;
            /*
             * other "nice-to-have"-functions:
             * 
            // append the function svf2 directly after this function. this->tf == svf2->t0 must hold
            virtual SingleVariableFunction* copyWithSeamlessAppend(SingleVariableFunction* svf2) const;
            
            // append the function svf2 to this function. this->tf == svf2->t0 is not required to hold. the values in 
            //  between are "zero", "constantLeft", "constantRight" or "linearInterp".
            virtual SingleVariableFunction* copyWithMergeInDomain(SingleVariableFunction* svf2, std::string connection) const;
            
            // the values of this function and svf2 are stacked [this->t0, this->tf] == [svf2->t0, svf2->tf] must hold
            virtual SingleVariableFunction* copyWithStackedValues(SingleVariableFunction* svf2) const;
            
            */
            
            PUBLIC_STREAM_SERIALIZATION_DECLARATION(SingleVariableFunction)
            
        private:
            friend class boost::serialization::access;
            
            template<class Archive>
            void serialize(Archive& ar, const unsigned int version) {
                ar & boost::serialization::base_object<Function>(*this);
                ar & t0;
                ar & tf;
            }
        };
        
        
}}



BOOST_CLASS_EXPORT_KEY(ofc::functionLib::SingleVariableFunction)
