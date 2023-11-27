
#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>

#include "SingleVariableFunction.h"


BOOST_CLASS_EXPORT_IMPLEMENT(ofc::functionLib::SingleVariableFunction)


namespace ofc {
namespace functionLib {
        
PUBLIC_STREAM_SERIALIZATION_DEFINITION(ofc::functionLib::SingleVariableFunction)



ofc::functionLib::SingleVariableFunction::SingleVariableFunction(size_t dimY, double t0, double tf)  : Function(1, dimY)
{
    this->t0 = t0;
    if(tf > t0){ // ensure that tf >= t0
        this->tf = tf;
    }
    else {
        this->tf = t0;
    }
}


ofc::functionLib::SingleVariableFunction::SingleVariableFunction(const ofc::functionLib::SingleVariableFunction& svf) : Function(1, svf.NY())
{
    t0 = svf.t0;
    tf = svf.tf;
}


ofc::functionLib::SingleVariableFunction::~SingleVariableFunction()
{ // nothing to do here
}


double ofc::functionLib::SingleVariableFunction::getT0() const
{
    return this->t0;
}


double ofc::functionLib::SingleVariableFunction::getTf() const
{
    return this->tf;
}


bool ofc::functionLib::SingleVariableFunction::evaluate(math::Real* result, const math::Real* x) const
{
    return false;
}


bool ofc::functionLib::SingleVariableFunction::evaluateDerivative(math::Real* result, const math::Real* x, size_t nx, size_t degree) const
{
    return false;
}


ofc::functionLib::SingleVariableFunction * ofc::functionLib::SingleVariableFunction::copy() const
{
    return new SingleVariableFunction(*this);
}


SingleVariableFunction* SingleVariableFunction::copyWithCroppedOutput() const
{
    return 0;
}


SingleVariableFunction* SingleVariableFunction::copyWithDomainMapping(double t0, double tf) const
{
    if(std::isnan(t0)) t0 = this->t0;
    if(std::isnan(tf)) tf = this->tf;
    if(t0 > tf || (t0 == tf && this->t0 < this->tf)) return 0;
    
    auto f2 = this->copy();
    f2->t0 = t0;
    f2->tf = tf;
    return f2;
}


SingleVariableFunction* SingleVariableFunction::copyWithDomainCut(double t0, double tf) const
{
    if(std::isnan(t0)) t0 = this->t0;
    if(std::isnan(tf)) tf = this->tf;
    if(t0 > tf || (t0 == tf && this->t0 < this->tf)) return 0;
        
    auto f2 = this->copy();
    f2->t0 = t0;
    f2->tf = tf;
    return f2;
}

SingleVariableFunction* SingleVariableFunction::copyWithDomainShift(const double offset) const
{
    auto f2 = this->copy();
    f2->t0 += offset;
    f2->tf += offset;
    return f2;
}


}}
