#pragma once 


#include <Eigen/Core>

#ifdef _WIN32
#define DIRCOL_INF (1e10 + 1)
#define DIRCOL_NEGINF (-1e10 - 1)
#endif

namespace ofc {
	namespace math {
		typedef double Real;
		
		typedef Eigen::Matrix<bool, (int)-1, (int)1> VectorXb;
		typedef Eigen::Matrix<double, int(-1), int(1)> TVectorX;
		typedef Eigen::Matrix<double, int(3), int(1)> TVector3;
		
        #ifndef _WIN32
		const double DIRCOL_INF = 1e10 + 1;
		const double DIRCOL_NEGINF = -1e10 - 1;
        #endif
		
		#ifdef _WIN32
		typedef unsigned int uint;
		#endif
	}
}
