
#include <utility>
#include <cmath>
#include <limits>
#include <boost/serialization/export.hpp>
#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>

#include "PiecewiseFunction.h"


BOOST_CLASS_EXPORT_IMPLEMENT(ofc::functionLib::PiecewiseFunction)

namespace ofc {
namespace functionLib{

    PUBLIC_STREAM_SERIALIZATION_DEFINITION(ofc::functionLib::PiecewiseFunction)

    PiecewiseFunction::PiecewiseFunction(size_t dimY) : SingleVariableFunction(dimY) {
      _Intervals.reserve(100);
      _Functions.reserve(100);
    }
    
    PiecewiseFunction::PiecewiseFunction(const PiecewiseFunction& pwfun) : SingleVariableFunction(pwfun.NY(), pwfun.t0, pwfun.tf) {
      _Intervals = pwfun._Intervals;
      _Functions.reserve(_Intervals.size());
      for(auto it = pwfun._Functions.begin(); it < pwfun._Functions.end(); it++){
	_Functions.push_back((*it)->copy());
      }
    }
    
    
    PiecewiseFunction::~PiecewiseFunction(){
      for(auto it = _Functions.begin(); it < _Functions.end(); ++it){
        delete (*it);
      }
    }

    PiecewiseFunction* PiecewiseFunction::copyWithCroppedOutput() const {
        if(this->NY()<= 1){
            return nullptr;
        }
        PiecewiseFunction* retval = new PiecewiseFunction(this->NY()-1);
        for(auto it : this->_Intervals) {
            retval->_Intervals.push_back(it);
        }
        for(auto it : this->_Functions){
            retval->_Functions.push_back(it->copyWithCroppedOutput());
        }
        retval->updateSingleVariableBounds();
        return retval;
    }
    
    
    
    PiecewiseFunction* PiecewiseFunction::copyWithDomainMapping(double t0, double tf) const 
    {
        if(std::isnan(t0)) t0 = this->t0;
        if(std::isnan(tf)) tf = this->tf;
        if(t0 > tf || (t0 == tf && this->t0 < this->tf)) return 0;
        
        PiecewiseFunction* f2 = this->copy();
        
        /*
         * m: [a1, b1] -> [a2, b2]
         * 
         * for x \in [a1, b]:
         * m(x) := a2 + (x - a1) * (b2-a2)/(b1-a1)
         *       = a2 - a1*(b2-a2)/(b1-a1) + x*(b2-a2)/(b1-a1)
         * 
         */
        double a2 = f2->getDefinitionInterval().a;
        double b2 = f2->getDefinitionInterval().b;
        
        double c2 = (tf-t0)/(b2-a2); // to save memory, a1,b1 is a2,b2 here and a2,b2 is t0,tf
        double c1 = t0 - a2*c2;
        
        for(size_t i = 0; i < this->_Functions.size()-1; ++i){
            a2 = c1 + f2->_Intervals[i].a*c2;
            b2 = c1 + f2->_Intervals[i].b*c2;
            f2->_Intervals[i].a = a2;
            f2->_Intervals[i].b = b2;
            auto tmp = f2->_Functions[i];
            f2->_Functions[i] = f2->_Functions[i]->copyWithDomainMapping(a2, b2);
            delete tmp;
        }
        { // the last interval has size 0 (i.e., [a,a]) and thus needs to be treated separately
            size_t i = this->_Functions.size()-1;
            auto tmp = f2->_Functions[i];
            f2->_Functions[i] = f2->_Functions[i]->copyWithDomainShift(tf - f2->_Intervals[i].b);
            delete tmp;
            f2->_Intervals[i].a = tf;
            f2->_Intervals[i].b = tf;
        }
        
        f2->updateSingleVariableBounds();
        return f2;
    }
    
    
    PiecewiseFunction* PiecewiseFunction::copyWithDomainCut(double t0, double tf) const 
    {
        if(t0 > tf || (t0 == tf && this->t0 < this->tf)) return 0;
        if(tf > this->tf) tf = this->tf;
        if(t0 < this->t0) t0 = this->t0;
        
        Interval value(t0, t0);
        size_t index;
        bool isEqual;
        
        PiecewiseFunction* f2 = this->copy();
        f2->t0 = t0;
        f2->tf = tf;
        
        if(!std::isnan(t0)) {
            if(find(&index, &isEqual, value) // find index of t0
            && index > 0 && _Intervals.back().b >= t0) {
                for(auto it = f2->_Functions.begin(); it < f2->_Functions.begin()+index-1; ++it){
                    delete (*it); // delete functions until the function with t0
                }
                f2->_Functions.erase(f2->_Functions.begin(), f2->_Functions.begin()+index-1); // clear the vectors
                f2->_Intervals.erase(f2->_Intervals.begin(), f2->_Intervals.begin()+index-1);
                if(!isEqual){ // if inside the interval, crop this interval
                    double b = f2->_Intervals[0].b;
                    f2->_Intervals.erase(f2->_Intervals.begin());
                    f2->_Intervals.insert(f2->_Intervals.begin(), Interval(t0, b));
                    
                    SingleVariableFunction* tmp = f2->_Functions.front();
                    f2->_Functions.erase(f2->_Functions.begin());
                    f2->_Functions.insert(f2->_Functions.begin(), tmp->copyWithDomainCut(t0, b));
                    delete tmp;
                }
            }
        }
                
        value.b = tf;
        value.a = tf;
        if(!std::isnan(tf)) {
            if(f2->find(&index, &isEqual, value)
            && index > 0 && _Intervals.back().b >= t0) {
                for(auto it = f2->_Functions.begin()+index; it < f2->_Functions.end(); ++it){
                    delete (*it); // delete functions until the function with t0
                }
                f2->_Functions.erase(f2->_Functions.begin()+index, f2->_Functions.end());
                f2->_Intervals.erase(f2->_Intervals.begin()+index, f2->_Intervals.end());
                // crop the last interval
                double a = f2->_Intervals.back().a;
                f2->_Intervals.pop_back();
                f2->_Intervals.push_back(Interval(a, tf));
                
                SingleVariableFunction* tmp = f2->_Functions.back();
                f2->_Functions.pop_back();
                f2->_Functions.push_back(tmp->copyWithDomainCut(a, tf));
                delete tmp;
            }
        }
        f2->updateSingleVariableBounds();
        return f2;
    }
    
    
    PiecewiseFunction* PiecewiseFunction::copyWithDomainShift(const double offset) const
    {
        PiecewiseFunction* f2 = this->copy();
        f2->t0 += offset;
        f2->tf += offset;
    
        for(size_t i = 0; i < this->_Functions.size(); ++i){
            auto tmp = f2->_Functions[i];
            f2->_Functions[i] = tmp->copyWithDomainShift(offset);
            delete tmp;
            f2->_Intervals[i].b += offset;
            f2->_Intervals[i].a += offset;
        }
        f2->updateSingleVariableBounds();
        return f2;
    }
    
    
  PiecewiseFunction* PiecewiseFunction::copy() const {
    return new PiecewiseFunction(*this);
  }

    
    bool PiecewiseFunction::add(math::Real lower, SingleVariableFunction* func){
        Interval I(lower, (math::Real)std::numeric_limits< math::Real >::max());
        size_t index;
        bool isEqual;
        if(find(&index, &isEqual, I)){
            if(isEqual){
                // interval alread exists
                return false;
            }
            else {
                if(index < _Intervals.size()){
                    // add some interval in between:
                    auto elt = _Intervals.begin()+index;
                    I.b = elt->b;
                    elt->b = I.a;
                    _Intervals.insert(elt, I);
                    _Functions.insert(_Functions.begin()+index, func);
                }
                else {
                    // is last element:
                    auto elt = _Intervals.begin()+(index-1);
                    I.b = I.a;
                    elt->b = I.a;
                    _Intervals.push_back(I);
                    _Functions.push_back(func);
                }
            }
        }
        else {
            // this is the first interval
            I.b = I.a;
            _Intervals.push_back(I);
            _Functions.push_back(func);
        }
        updateSingleVariableBounds();
        return true;
    }
    
    
    void PiecewiseFunction::updateSingleVariableBounds(){
        Interval I = getDefinitionInterval();
        this->t0 = I.a;
        this->tf = I.b;
    }


    Interval PiecewiseFunction::getDefinitionInterval() const{
      Interval interval(_Intervals.begin()->a, _Intervals.back().b);
      return interval;
    }

    bool PiecewiseFunction::evaluate(math::Real* result, const math::Real* x) const{
      Interval value(x[0], x[0]);
      size_t index;
      bool isEqual;
      if(find(&index, &isEqual, value)
      && index > 0 && _Intervals.back().b >= x[0]) {
        return _Functions[index-1]->evaluate(result, x);
      }
      else {
        return false;
      }
    }
    
    bool PiecewiseFunction::evaluateDerivative(math::Real* result, const math::Real* x, size_t nx, std::size_t degree) const {
      size_t index;
      bool isEqual;
      bool succ = true;
      for(size_t i = 0; i < nx; ++i){
        Interval value(x[i], x[i]);
        if(find(&index, &isEqual, value) && index > 0 && _Intervals.back().b >= x[i]) {
            if(_Intervals[index-1].length() == 0 && index > 1 && _Intervals[index-2].b == x[i]){ // if interval length is 0 and there is some other interval adjacent to the left
                succ &= _Functions[index-2]->evaluateDerivative(result+(this->_dimY*i), x+i, 1, degree);
            }
            else {
                succ &= _Functions[index-1]->evaluateDerivative(result+(this->_dimY*i), x+i, 1, degree);
            }
        }
        else {
            return false;
        }
      }
      return succ;
    }
        
  
    bool PiecewiseFunction::find(size_t* index, bool* isEqual, const Interval& value) const{
      size_t len = _Intervals.size();
	  *isEqual = false;
      if(len < 1) 
	return false; // no element found
      size_t A, B, tmp;
      // initialisation:
      A = 0;
      B = len;
      // search loop:
      while(B-A > 1){
	tmp = (size_t) ( A + floor((B-A)/2.0) );
	if(value.a < _Intervals[tmp].a){
	  B = tmp;
	}
	else {
	  A = tmp;
	}
      }
      // check, if A==0 holds:
      if( A == 0 && value.a < _Intervals[0].a ){
	(*isEqual) = false;
	(*index) = 0;
	return true;
      }
      // check, if equality holds:
      if( _Intervals[A].a == value.a ){
	(*isEqual) = true;
      }
      (*index) = A+1;
      return true;      
    }

  
}
}
