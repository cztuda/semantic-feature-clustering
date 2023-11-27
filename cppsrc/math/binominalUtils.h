#pragma once

#include <vector>

namespace ofc {
	namespace math {

		/**
		* Inplace change A to the next line of the Pascal triangle.
		*/
		void stepBinomialCoeff(std::vector<int>& A)
		{
			if (!A.empty()) {
				int tmp; // keep the number that just has been overwritten
				for (auto it = A.begin() + 1; it < A.end(); ++it) {
					tmp = *it;
					*it = *(it - 1) + tmp; // sum current and previous entry
				}
			}
			A.push_back(1);
		}

	}
}