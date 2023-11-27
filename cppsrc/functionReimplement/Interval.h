#pragma once

#include <limits>
// #include <boost/archive/text_iarchive.hpp>
// #include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>

#include "../math/definitions.h"
#include "../serialization/serialization_helper.h"

namespace ofc {
namespace functionLib{
    
  class Interval{
  public:
    math::Real a;
    math::Real b;
    // set the interval to the invalid values (+inf, -inf)
    // which is useful for detrmining the max and min values
    void setInvalid(){
      b = -std::numeric_limits<math::Real>::max();
      a = std::numeric_limits<math::Real>::max();
    }

    /**
    * \brief Sets the interval to (-inf,inf) .
    */
    void setUnbounded(){
      a = -std::numeric_limits<math::Real>::max();
      b = std::numeric_limits<math::Real>::max();
    }

    /**
    * \brief Extends interval to contain value.
    *
    * \author Tobi
    * \date 30.05.2012
    *
    * \param value The value.
    */
    inline void extendTo(math::Real value){
      if(value < a)a = value;
      if(value > b)b  =value;
    }
    Interval(const math::Real low, const math::Real high):a(low), b(high){}
    Interval(){
      setUnbounded();
    }
    Interval(math::Real value): a(value), b(value) {}

    Interval( std::istream& s){
	    math::Real low, high;
	    s >> low;
	    s >> high;
	    a = low;
	    b = high;
    }
    
    Interval(const Interval& I) : a(I.a), b(I.b) {}
    ~Interval(){}

    bool overlaps(const Interval & other){
      if(other.b < a)return false;
      if(other.a > b)return false;
      return true;
    }
    bool disjoint(const Interval & other){
      return !overlaps(other);
    }
    bool isSubsetOf(const Interval & other){
      if(other.a <= a && other.b >= b)return true;
      return false;
    }
    bool isBefore(math::Real value)const{
      if(b < value)return true;
      return false;
    }
    bool isAfter(math::Real value)const{
      if(value < a)return true;
      return false;
    }
    bool isElementOf(math::Real value)const{
      if(a <= value && value <= b)return true;
      return false;
    }

    inline math::Real length()const{
      return b-a;
    }

    friend bool operator == (const Interval & a, const Interval &  b){
      return a.a==b.a&&a.b==b.b;
    }

    void serialize(std::ostream& s) const;
    void unserialize(std::istream& s);
    
  private:
      friend class boost::serialization::access;
      
      template<class Archive>
      void serialize(Archive& ar, const unsigned int version) {
          ar & a;
          ar & b;
      }
  };



}
}

BOOST_CLASS_EXPORT_KEY(ofc::functionLib::Interval)
