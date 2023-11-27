#pragma once

#include <vector>
#include "../math/definitions.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace ofc::functionLib {

struct PiecewiseFunctionData {
    uint n;
    uint np;
    uint nphases;
    char estadj;
    double t0;
    double tf;
    std::vector<uint> ngridpts;
    std::vector<std::string> names;
    std::vector<ofc::math::Real> doubles;
    std::vector<ofc::math::Real> parameter;
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int)
    {
        ar & n;
        ar & np;
        ar & nphases;
        ar & estadj;
        ar & t0;
        ar & tf;
        ar & ngridpts;
        ar & names;
        ar & doubles;
        ar & parameter;
    }
  };

}
