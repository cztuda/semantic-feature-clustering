#include "eigensparseutils.h"

void ofc::util::getMatrixData(const Eigen::SparseMatrix<double>& mat, std::vector<double>& values, std::vector<double>& rows, std::vector<double>& columns){
        
        for (int k=0; k<mat.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
            {
                values.push_back(it.value());
                rows.push_back(it.row());
                columns.push_back(it.col());
            }
    }
    
