
#include <cmath>
#include <cstring>
#include <omp.h>
#include <boost/serialization/export.hpp>
#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>

#include "Polynom.h"
#include "../math/binominalUtils.h"


BOOST_CLASS_EXPORT_IMPLEMENT(ofc::functionLib::Polynom)
// BOOST_CLASS_EXPORT_IMPLEMENT(ofc::functionLib::NormalizedPolynom)

using namespace ofc::math;

namespace ofc {
namespace functionLib {   
  
    PUBLIC_STREAM_SERIALIZATION_DEFINITION(ofc::functionLib::Polynom)

  Polynom::Polynom(size_t dimY, const std::vector<Real>* coefficients) : Polynom(dimY, (size_t)((double)coefficients->size() / dimY)){
    if(_dimY*degreeP1 != coefficients->size() || !setCoefficients(coefficients)){
      std::cerr << "Polynom: Given dimension and coefficients are not consistent. " << std::endl;
    }
  }
  
  /**
   * dimY = dimension of output
   * degreePlusOne = dimension of the polynomial (-> dimY*(degreePlusOne) coefficients are required)
   */
  Polynom::Polynom(size_t dimY, size_t degreePlusOne) : SingleVariableFunction(dimY){
    if(degreePlusOne > 100) degreePlusOne = 0;
    this->degreeP1 = degreePlusOne;
    if(dimY > 0){
      _Coefficients = new Real[degreePlusOne*_dimY];
      for(size_t i = 0; i < degreePlusOne*_dimY; i++) _Coefficients[i] = 0;
    }
  }
  
  Polynom::Polynom() : SingleVariableFunction(0), _Coefficients(0), degreeP1(0) {}
  
  Polynom::~Polynom(){
    delete [] _Coefficients;
  }
  
  Polynom::Polynom(const Polynom& p) : Polynom(p._dimY, p.degreeP1) {
      this->t0 = p.t0;
      this->tf = p.tf;
    for(size_t i = 0; i < degreeP1*_dimY; i++) _Coefficients[i] = p._Coefficients[i];
  }

  Polynom* Polynom::copy() const {
    return new Polynom(*this);
  }

  
  functionLib::Polynom* Polynom::copyWithCroppedOutput() const
{
    // create copy using virtual function (to account for NormalizedPolynom) and change dimY manually:
    Polynom* retval = this->copy();
    retval->_dimY -= 1;
    delete [] retval->_Coefficients;
    retval->_Coefficients = new Real[degreeP1*_dimY];
    
    // copy values from this->_Coefficients to retval->_Coefficients:
      for(size_t i = 0; i < this->degreeP1; ++i){
          memcpy(retval->_Coefficients + i*retval->NY(), this->_Coefficients + i*this->NY(), sizeof(math::Real)*(this->NY()-1));
      }
      return retval;      
  }

  
  void Polynom::domainMapping(Polynom* p, double t0, double tf, const double* coeffOld){
      double* coeffNew = p->_Coefficients;
      for(size_t i = 0; i < p->degreeP1*p->_dimY; ++i){ // initialize/reset new coefficients
          coeffNew[i] = 0;
      }
      
      double gamma = t0*(p->tf-p->t0)/(tf-t0) + p->t0;
      double mue = (p->tf-p->t0)/(tf-t0); // map:[t0, tf]->[p->t0,p->tf], map(x) := gamma + mue*x
      std::vector<int> Bfac;
      std::vector<double> gammaFactors(p->degreeP1); // = [1 gamma gamma^2 gamma^3 ...]
      std::vector<double> mueFactors(p->degreeP1); // = [1 mue mue^2 mue^3 ...]
      
      // prepare factors
      gammaFactors[0] = 1;
      mueFactors[0] = 1;
      for(size_t i = 1; i < p->degreeP1; ++i){
          gammaFactors[i] = gammaFactors[i-1]*gamma;
          mueFactors[i] = mueFactors[i-1]*mue;
      }
      
      // -> coefficients = [a_1 a_2 b_1 b_2 c_1 c_2]
      size_t K = p->_dimY;
      double factor;
      
      // iterate over all coefficients and replace coeff*x with coeff*(gamma+mue*x)^i:
      for(size_t i = 0; i < p->degreeP1-1; ++i){
          ofc::math::stepBinomialCoeff(Bfac);
          // factor out (gamma+mue*x)^i and add to coefficients:
          for(size_t j = 0; j <= i; ++j) {
              factor = gammaFactors[i-j] * mueFactors[j] * Bfac[j];
              for(size_t k = 0; k < K; ++k) {
                  coeffNew[j*K+k] += coeffOld[i*K+k]*factor;
              }
          }
      }
  }
  
  
  Polynom* Polynom::copyWithDomainMapping(double t0, double tf) const
  {
      Polynom* f2 = this->copy();
      Polynom::domainMapping(f2, t0, tf, this->_Coefficients);
      return f2;
  }
      
      
    Polynom* Polynom::copyWithDomainCut(double t0, double tf) const
    {
        if(std::isnan(t0)) t0 = this->t0;
        if(std::isnan(tf)) tf = this->tf;
        if(t0 > tf || (t0 == tf && this->t0 < this->tf)) return 0;
        
        Polynom* f2 = this->copy();
        f2->t0 = t0;
        f2->tf = tf;
        return f2;
    }
    
    
    Polynom* Polynom::copyWithDomainShift(const double offset) const
    {
        return this->copyWithDomainMapping(this->t0+offset, this->tf+offset);
    }
  
  
  bool Polynom::evaluate(math::Real* result, const math::Real* x) const
{
    memcpy(result, &_Coefficients[(degreeP1-1)*_dimY], sizeof(Real)*_dimY);
    for(size_t deg = 1; deg < degreeP1; deg++){
      factorInplace(result, *x, _dimY);
      addInplace(result, &_Coefficients[_dimY*(degreeP1-1-deg)], _dimY);
    }
    return true;
  }
  
  
  
  bool Polynom::evaluate(Real* result, const Real* x, size_t nx) const{
    for(size_t i = 0; i < nx; i++){
      memcpy(&result[_dimY*i], &_Coefficients[(degreeP1-1)*_dimY], sizeof(Real)*_dimY);
    }
    for(size_t deg = 1; deg < degreeP1; deg++){
      for(size_t i = 0; i < nx; i++){
	factorInplace(&result[_dimY*i], x[i], _dimY);
	addInplace(&result[_dimY*i], &_Coefficients[_dimY*(degreeP1-1-deg)], _dimY);
      }
    }
    return true;
  }
  
  
  Real* Polynom::evaluate(const Real* x, size_t nx) const{
    Real* result = new Real[nx*_dimY];
    if(evaluate(result, x, nx)){
      return result;
    }
    return 0;
  }
  
  inline int mult(int a, int b) {
    if(b < a) return 0;
    int result = a;
    for(int i = a+1; i <= b; ++i) {
      result *= i;
    }
    return result;
  }
  
  bool Polynom::evaluateDerivative(Real* result, const Real* x, size_t nx, size_t degree) const {
    if(degree >= this->degreeP1) {
      for(size_t i = 0; i < nx*_dimY; ++i) {
	result[i] = 0;
      }
      return true;
    }
    if(degree == 0) {
      return this->evaluate(result, x, nx);
    }
    
    // degreeP1 must be at least 2, otherwise, the previous if-conditions would have been fulfilled
    Real *tmp = new Real[_dimY];
    
    memcpy(tmp, &_Coefficients[(degreeP1-1)*_dimY], sizeof(Real)*_dimY);
    factorInplace(tmp, mult(degreeP1-degree, degreeP1-1), _dimY);
    for(size_t i = 0; i < nx; i++){
      memcpy(&result[_dimY*i], tmp, sizeof(Real)*_dimY);
    }
    
    for(size_t deg = degreeP1-2; deg >= degree; --deg){
      memcpy(tmp, &_Coefficients[deg*_dimY], sizeof(Real)*_dimY);
      factorInplace(tmp, mult(deg-degree+1, deg), _dimY);
      for(size_t i = 0; i < nx; i++){
	factorInplace(&result[_dimY*i], x[i], _dimY);
	
	addInplace(&result[_dimY*i], tmp, _dimY);
      }
    }
    delete [] tmp;
    return true;
  }
  
  
bool Polynom::evaluateCoefficientDerivative(Real* result, const Real x) const {
  result[0] = 1.0;
  Real tmp = x;
  for(size_t i = 1; i < degreeP1; ++i) {
    result[i] = tmp;
    tmp *= x;
  }
  return true;
}

  
  bool Polynom::evaluate(std::vector<Real>* result, const Real* x, size_t nx) const{
    result->reserve(nx*_dimY);
    return evaluate(result->data(), x, nx);
  }
  
  const Real * Polynom::Coefficients() const{
    return _Coefficients;
  }
  
  
  
  bool Polynom::setCoefficients(const std::vector<Real>*  coeffs){
    if(coeffs->size() < degreeP1*_dimY){
      memcpy(_Coefficients, coeffs->data(), sizeof(Real)*coeffs->size());
      return false;
    }
    else {
      memcpy(_Coefficients, coeffs->data(), sizeof(Real)*degreeP1*_dimY);
      return (coeffs->size() == degreeP1*_dimY);
    }
  }
  
  bool Polynom::setCoefficient(Real value, size_t index){
    if(index < degreeP1*_dimY){
      _Coefficients[index] = value;
      return true;
    }
    return false;
  }
      
  size_t Polynom::getDegree(){
    return degreeP1-1;
  }

  
  
  inline void Polynom::factorInplace(Real* a, Real b, size_t n) const{
    #pragma omp simd
    for(size_t i = 0; i < n; i++){
      a[i] *= b;
    }
  }
  inline void Polynom::addInplace(Real* a, Real* b, size_t n) const {
    #pragma omp simd
    for(size_t i = 0; i < n; i++){
      a[i] += b[i];
    }
  }

  
}
}

