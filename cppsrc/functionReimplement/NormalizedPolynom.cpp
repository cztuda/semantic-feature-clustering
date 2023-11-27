
#include <cmath>
#include <cstring>
#include <omp.h>
#include <boost/serialization/export.hpp>
#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>

#include "NormalizedPolynom.h"

BOOST_CLASS_EXPORT_IMPLEMENT(ofc::functionLib::NormalizedPolynom)

using namespace ofc::math;

namespace ofc {
	namespace functionLib {

		PUBLIC_STREAM_SERIALIZATION_DEFINITION(ofc::functionLib::NormalizedPolynom)

			NormalizedPolynom::NormalizedPolynom(size_t dimY, const std::vector<Real>* coefficients, const Interval& interval) : Polynom(dimY, coefficients) {
			setInterval(interval);
		}

		NormalizedPolynom::NormalizedPolynom(size_t dimY, size_t degree) : Polynom(dimY, degree) {
			setInterval(Interval(0, 0));
		}

		NormalizedPolynom::NormalizedPolynom() : Polynom() {
			t0 = 0;
			tf = 0;
		}

		NormalizedPolynom::NormalizedPolynom(const NormalizedPolynom& np) : NormalizedPolynom(np._dimY, np.degreeP1) {
			for (size_t i = 0; i < degreeP1*_dimY; i++) _Coefficients[i] = np._Coefficients[i];
			setInterval(np.getInterval());
		}


		NormalizedPolynom* NormalizedPolynom::copy() const {
			return new NormalizedPolynom(*this);
		}



		NormalizedPolynom* NormalizedPolynom::copyWithDomainMapping(double t0, double tf) const
		{
			if (std::isnan(t0)) t0 = this->t0;
			if (std::isnan(tf)) tf = this->tf;
			if (t0 > tf || (t0 == tf && this->t0 < this->tf)) return 0;

			NormalizedPolynom* f2 = this->copy();
			f2->t0 = t0;
			f2->tf = tf;
			return f2;
		}


		NormalizedPolynom* NormalizedPolynom::copyWithDomainCut(double t0, double tf) const
		{
			if (std::isnan(t0)) t0 = this->t0;
			if (std::isnan(tf)) tf = this->tf;
			if (t0 > tf || (t0 == tf && this->t0 < this->tf)) return 0;

			// perform domain cut:
			NormalizedPolynom* f2 = this->copy();
			f2->t0 = t0;
			f2->tf = tf;

			// perform domain mapping back onto [0, 1]
			NormalizedPolynom::domainMapping(f2, 0, 1, this->_Coefficients);

			return f2;
		}


		NormalizedPolynom* NormalizedPolynom::copyWithDomainShift(const double offset) const
		{
			NormalizedPolynom* f2 = this->copy();
			f2->t0 += offset;
			f2->tf += offset;
			return f2;
		}


		bool NormalizedPolynom::evaluate(Real* result, const Real* x) const {
			Real x2 = (x[0] - t0) / (tf - t0);
			return Polynom::evaluate(result, &x2);
		}

		bool NormalizedPolynom::evaluate(Real* result, const Real* x, size_t nx) const {
			double intervalLength = tf - t0;
			for (size_t i = 0; i < nx; i++) {
				if (!Polynom::evaluate(&result[_dimY*i], (x[i] - t0) / intervalLength)) return false;
			}
			return true;
		}

		Real* NormalizedPolynom::evaluate(const Real* x, size_t nx) const {
			Real* result = new Real[nx*_dimY];
			double intervalLength = tf - t0;
			for (size_t i = 0; i < nx; i++) {
				if (!Polynom::evaluate(&result[_dimY*i], (x[i] - t0) / intervalLength)) {
					delete[] result;
					return 0;
				}
			}
			return result;
		}

		bool NormalizedPolynom::evaluate(std::vector<Real>* result, const Real* x, size_t nx) const {
			result->reserve(nx*_dimY);
			Real x2;
			double intervalLength = tf - t0;
			for (size_t i = 0; i < nx; i++) {
				x2 = (x[i] - t0) / intervalLength;
				if (!Polynom::evaluate(&result[_dimY*i], &x2)) return false;
			}
			return true;
		}


		bool NormalizedPolynom::evaluateDerivative(Real* result, const Real* x, std::size_t nx, std::size_t degree) const {
			Real tmpX;
			double intervalLength = tf - t0;
			for (size_t i = 0; i < nx; i++) {
				tmpX = (x[i] - t0) / intervalLength;
				if (!Polynom::evaluateDerivative(&result[_dimY*i], &tmpX, 1, degree)) {
					return false;
				}
				for (size_t j = 0; j < _dimY; ++j) {
					result[_dimY*i + j] /= intervalLength;
				}
			}
			return true;
		}


		Interval NormalizedPolynom::getInterval() const {
			if ((tf - t0) < 1e-6) {
				return Interval(t0, t0);
			}
			else {
				return Interval(t0, tf);
			}
		}

		void NormalizedPolynom::setInterval(const Interval& interval) {
			tf = interval.b;
			t0 = interval.a;
		}



	}
}
