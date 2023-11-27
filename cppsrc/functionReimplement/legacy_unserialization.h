#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../util/exceptions.h"
#include "functionLibSerialization.h"
#include "Interval.h"
#include "Function.h"
#include "PiecewiseFunction.h"
#include "Polynom.h"
#include "NormalizedPolynom.h"

namespace ofc::functionLib::legacy {
    
    
    
    /***** Function.h ******/
    
    ofc::functionLib::Function* unserialize_function(std::istream& s) {
        double _dimX, _dimY;
        s >> _dimX;
        s >> _dimY;
        return new ofc::functionLib::Function(_dimX, _dimY);
    }
    
    ofc::functionLib::Function* constructor_Function(std::istream& s) {
        ofc::functionLib::Function* f;
        std::string tmp;
        s >> tmp;
        if(tmp.compare("(funct")){ throw ofc::exception::InconsistentInputError("Given stream is not a serialized function."); }
        f = unserialize_function(s);
        s >> tmp;
        return f;
    }
    
    
    /***** Interval.h ******/
    
    ofc::functionLib::Interval unserialize_interval(std::istream& s){
        double a,b;
        s >> a;
        s >> b;
        return ofc::functionLib::Interval(a,b);
    }
    
    
    /***** PiecewiseFunction.cpp ******/
    
    ofc::functionLib::PiecewiseFunction* unserialize_piecewiseFunction(std::istream& s, const size_t _dimY){
        std::string word;
        size_t len;
        s >> len;
        std::vector<ofc::functionLib::Interval> _Intervals;
        std::vector<ofc::functionLib::SingleVariableFunction*> _Functions;
        _Intervals.reserve(len);
        _Functions.reserve(len);
        for(size_t it = 0; it < len; ++it){
            _Intervals.push_back(unserialize_interval(s));
        }
        ofc::functionLib::SingleVariableFunction* svfun;
        for(size_t it = 0; it < len; ++it){
            s >> word;
            svfun = dynamic_cast<ofc::functionLib::SingleVariableFunction*>(unserializeFunction(word, s));
            if(svfun == nullptr) { throw ofc::exception::FileFormatError("Function read in PiecewiseFunction must be a single variable function but it is not."); }
            _Functions.push_back(svfun);
        }   
        
        ofc::functionLib::PiecewiseFunction* f = new ofc::functionLib::PiecewiseFunction(_dimY);
        for(size_t it = 0; it < len; ++it){
            f->add(_Intervals[it].a, _Functions[it]);
        }
        f->updateSingleVariableBounds();
        return f;
    }
    
    ofc::functionLib::PiecewiseFunction* constructor_PiecewiseFunction(std::istream& s) {
        std::string tmp;
        size_t _dimY;
        s >> tmp;
        if(tmp.compare("(pwfun")){ throw ofc::exception::InconsistentInputError("Given stream is not a serialized piecewise function."); }
        s >> _dimY;
        ofc::functionLib::PiecewiseFunction* f = unserialize_piecewiseFunction(s, _dimY);
        s >> tmp;
        return f;
    }
    
    
    /***** Polynom.cpp ******/
    
    ofc::functionLib::Polynom* unserialize_polynom(std::istream& s, const size_t _dimY, const size_t degreeP1){
        std::vector<double> _Coefficients(_dimY*degreeP1);
        for(size_t i = 0; i < _dimY*degreeP1; ++i){
            s >> _Coefficients[i];
        }
        return new ofc::functionLib::Polynom(_dimY, &_Coefficients);
    }
    
    ofc::functionLib::Polynom* constructor_Polynom(std::istream& s){
        std::string tmp;
        s >> tmp;
        if(tmp.compare("(polyn")){ throw ofc::exception::InconsistentInputError("Given stream is not a serialized polynom."); }
        size_t _dimY, degreeP1;
        s >> _dimY;
        s >> degreeP1;
        ofc::functionLib::Polynom* f = unserialize_polynom(s, _dimY, degreeP1);
        s >> tmp;
        return f;
    }  
    
    
    ofc::functionLib::NormalizedPolynom* unserialize_normalizedPolynom(std::istream& s, const size_t _dimY, const size_t degreeP1){
        double intervalLength, intervalA;
        bool isSingular;
        s >> intervalLength;
        s >> intervalA;
        s >> isSingular;
        ofc::functionLib::Interval I(intervalA, intervalA+intervalLength);
        
        ofc::functionLib::Polynom* f = unserialize_polynom(s, _dimY, degreeP1);
        
        const double* coeffs = f->Coefficients();
        std::vector<double> coeffVector(coeffs, coeffs+(_dimY*degreeP1));
        return new ofc::functionLib::NormalizedPolynom(_dimY, &coeffVector, I);
    }
    
    ofc::functionLib::NormalizedPolynom* constructor_NormalizedPolynom(std::istream& s) {
        std::string tmp;
        s >> tmp;
        if(!tmp.compare("(npoly")){ throw ofc::exception::InconsistentInputError("Given stream is not a serialized normalized polynom."); }
        size_t _dimY, degreeP1;
        s >> _dimY;
        s >> degreeP1;
        
        ofc::functionLib::NormalizedPolynom* f = unserialize_normalizedPolynom(s, _dimY, degreeP1);
        s >> tmp;
        return f;
    }
    
    
}      
