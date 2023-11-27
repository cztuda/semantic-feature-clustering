#pragma once

#include <string>

#include "piecewisefunctiondata.h"
#include "../math/definitions.h"
#include "PiecewiseFunction.h"

using namespace ofc::math;

namespace ofc {
namespace functionLib{
  
  class TrajectoryReader {
  public:
    // reads a trajectory.  the format of the stream must be as used in DirCol
    bool read(std::istream & stream, functionLib::PiecewiseFunction** trajectory=0, PiecewiseFunctionData* dataExport=0);
  private:

    bool parseCubicPiecewiseFunction(functionLib::PiecewiseFunction * piecewise, std::vector<Real> & doubles, uint dim);
    bool parseLinearPiecewiseFunction(functionLib::PiecewiseFunction * piecewise, std::vector<Real> & doubles, uint dim);
  };
}
}

