#pragma once

#include <Eigen/Sparse>
#include <vector>

namespace ofc::util{
    
    void getMatrixData(const Eigen::SparseMatrix<double>& mat, std::vector<double>& values, std::vector<double>& rows, std::vector<double>& columns);
    
}
