#pragma once

#include "../serialization/serialization_helper.h"
#include <boost/serialization/vector.hpp>

#include "SingleVariableFunction.h"
#include "Interval.h"
#include "../math/definitions.h"

namespace ofc {
namespace functionLib {   
  
class Polynom : public SingleVariableFunction {
protected:
  math::Real* _Coefficients;
  size_t degreeP1; //degree+1
public:
  /**
    * Create polynomial. Coefficients are give in COLUMN-MAJOR format,
    * i.e., a for [x_1; x_2; ... x_n], then b for [x_1; x_2; ... x_n], etc. 
    * The given coefficients are copied into the class.
    * 
    * EXAMPLE:
    * second order polynomial a + b*x + c*x^2 with two-dimensional output:
    * matrix = [a_1  b_1  c_1
    * 		 a_2  b_2  c_2]
    * -> coefficients = [a_1 a_2 b_1 b_2 c_1 c_2]
    * 
    */
  Polynom(size_t dimY, const std::vector<math::Real>* coefficients);
  
  Polynom(size_t dimY, size_t degreePlusOne);
    
  Polynom(const Polynom& p);
  
  virtual ~Polynom();
  
  
  /**
    * It is assumed the the allocated memory results points to is sufficiently large
    * (at least sizeof(math::Real)*nx*_dimY) that the result can be written to that adress
    * 
    */
  virtual bool evaluate(math::Real* result, const math::Real* x) const override;
  
  
  /**
    * It is assumed the the allocated memory results points to is sufficiently large
    * (at least sizeof(math::Real)*nx*_dimY) that the result can be written to that adress
    * 
    */
  virtual bool evaluate(math::Real* result, const math::Real* x, size_t nx) const override;
  
  /**
    * Allocates the space for the result and returns the pointer to the memory.
    * The result needs to be deallocated externally     * 
    * 
    */
  virtual math::Real* evaluate(const math::Real* x, size_t nx=1) const;
  
  /**
    * 
    */
  virtual bool evaluate(std::vector<math::Real>* result, const math::Real* x, size_t nx=1) const;
  
  /**
    * It is assumed the the allocated memory results points to is sufficiently large
    * (at least sizeof(math::Real)*nx*_dimY) that the result can be written to that adress
    * 
    */
  virtual bool evaluateDerivative(math::Real* result, const math::Real* x, size_t nx, size_t degree) const override;
  
  /**
    * It is assumed the the allocated memory results points to is sufficiently large
    * (at least sizeof(math::Real)*(degree()+1)) that the result can be written to that adress
    * Return value is given in COLUMN-MAJOR format and can be interpreted as follows:
    * result = [dp(x)/da, dp(x)/db, dp(x)/dc, ...] = [1, x, x^2, ...]
    */
  virtual bool evaluateCoefficientDerivative(math::Real* result, const math::Real x) const;
  
  const math::Real * Coefficients() const;
  
  bool setCoefficients(const std::vector<math::Real>*  coeffs);
  
  bool setCoefficient(math::Real value, size_t index);
      
  size_t getDegree();
  
  virtual Polynom* copyWithCroppedOutput() const override;
    virtual Polynom* copyWithDomainMapping(double t0=NAN, double tf=NAN) const override;
    virtual Polynom* copyWithDomainCut(double t0=NAN, double tf=NAN) const override;
    virtual Polynom* copyWithDomainShift(const double offset) const override;
  
  PUBLIC_STREAM_SERIALIZATION_DECLARATION(ofc::functionLib::Polynom)
  
  virtual Polynom* copy() const override;
  
protected:
  inline void factorInplace(math::Real* a, math::Real b, size_t n) const;
  inline void addInplace(math::Real* a, math::Real* b, size_t n) const ;
  static void domainMapping(Polynom* p, double t0, double tf, const double* coeffOld);
  
    Polynom();
    
private:
    friend class boost::serialization::access;
    
    template<class Archive>
    void save(Archive & ar, const unsigned int version) const
    {
        ar & boost::serialization::base_object<SingleVariableFunction>(*this);
        ar & degreeP1;
        ar & boost::serialization::make_array<math::Real>(_Coefficients, degreeP1*_dimY);
    }
    
    template<class Archive>
    void load(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<SingleVariableFunction>(*this);
        ar & degreeP1;
        if(_Coefficients) {
            delete [] _Coefficients;
        }
        _Coefficients = new math::Real[degreeP1*_dimY];
        ar & boost::serialization::make_array<math::Real>(_Coefficients, degreeP1*_dimY);
    }
    
    BOOST_SERIALIZATION_SPLIT_MEMBER()
    
};
 
  
  
}
}

BOOST_CLASS_EXPORT_KEY(ofc::functionLib::Polynom)
