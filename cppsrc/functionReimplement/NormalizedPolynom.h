
#pragma once

#include "../serialization/serialization_helper.h"
#include <boost/serialization/vector.hpp>

#include "Polynom.h"
#include "Interval.h"
#include "../math/definitions.h"

namespace ofc {
	namespace functionLib {

		class NormalizedPolynom : public Polynom {

		public:
			NormalizedPolynom(size_t dimY, const std::vector<math::Real>* coefficients, const Interval& interval);

			NormalizedPolynom(size_t dimY, size_t degree);

			NormalizedPolynom(const NormalizedPolynom& np);

			virtual bool evaluate(math::Real* result, const math::Real* x) const override;

			virtual bool evaluate(math::Real* result, const math::Real* x, size_t nx) const override;

			virtual math::Real* evaluate(const math::Real* x, size_t nx = 1) const override;

			virtual bool evaluate(std::vector<math::Real>* result, const math::Real* x, size_t nx = 1) const override;

			virtual bool evaluateDerivative(math::Real* result, const math::Real* x, size_t nx, size_t degree) const override;

			Interval getInterval() const;

			void setInterval(const Interval& interval);

			PUBLIC_STREAM_SERIALIZATION_DECLARATION(ofc::functionLib::NormalizedPolynom)

			virtual NormalizedPolynom* copy() const override;
			virtual NormalizedPolynom* copyWithDomainMapping(double t0 = NAN, double tf = NAN) const override;
			virtual NormalizedPolynom* copyWithDomainCut(double t0 = NAN, double tf = NAN) const override;
			virtual NormalizedPolynom* copyWithDomainShift(const double offset) const override;

		private:
			friend class boost::serialization::access;

			template<class Archive>
			void serialize(Archive& ar, const unsigned int version) {
				ar & boost::serialization::base_object<Polynom>(*this);
			}
		protected:
			NormalizedPolynom();
		};

	}
}

BOOST_CLASS_EXPORT_KEY(ofc::functionLib::NormalizedPolynom)
