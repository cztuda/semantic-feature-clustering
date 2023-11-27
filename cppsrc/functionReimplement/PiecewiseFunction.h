#pragma once

#include "../serialization/serialization_helper.h"
#include <boost/serialization/vector.hpp>

#include "SingleVariableFunction.h"
#include "Interval.h"

namespace ofc {
namespace functionLib{
  
  class PiecewiseFunction : public SingleVariableFunction {
  
  private:
      std::vector<Interval> _Intervals;
      std::vector<SingleVariableFunction*> _Functions;
      
  public:

    PiecewiseFunction() : PiecewiseFunction(1){};
    
    PiecewiseFunction(size_t dimY);
    
    PiecewiseFunction(const PiecewiseFunction& pwfun);
    
    virtual ~PiecewiseFunction();
    
    bool add(math::Real lower, SingleVariableFunction* func);
    void updateSingleVariableBounds();

    Interval getDefinitionInterval() const;

    bool evaluate(math::Real* result, const math::Real* x) const override;
    
    virtual bool evaluateDerivative(math::Real* result, const math::Real* x, size_t nx, size_t degree) const override;
        
    virtual PiecewiseFunction* copy() const override;
    
    virtual PiecewiseFunction* copyWithCroppedOutput() const override;
    virtual PiecewiseFunction* copyWithDomainMapping(double t0=NAN, double tf=NAN) const override;
    virtual PiecewiseFunction* copyWithDomainCut(double t0=NAN, double tf=NAN) const override;
    virtual PiecewiseFunction* copyWithDomainShift(const double offset) const override;
    
    PUBLIC_STREAM_SERIALIZATION_DECLARATION(ofc::functionLib::PiecewiseFunction)
    
  private:
    friend class boost::serialization::access;
    /**
     * Find specific interval, using a custom comparison function
     * 
     * @param index the index in the vector, such that index-1 <= value. If index is 0, than value comes before index=0.
     * @param isEqual is true, if some existing element is matched exactly, else false
     * @param value the value to find
     * @return true, if success, else false
     * 
     */
    bool find(size_t* index, bool* isEqual, const Interval& value) const;
    
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & boost::serialization::base_object<SingleVariableFunction>(*this);
      ar & _Intervals;
      ar & _Functions;
    }
  };
}
}

BOOST_CLASS_EXPORT_KEY(ofc::functionLib::PiecewiseFunction)
