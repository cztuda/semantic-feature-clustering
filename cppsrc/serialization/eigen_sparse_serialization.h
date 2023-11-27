#pragma once

#include <Eigen/Sparse>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost::serialization {
    
    // ********** Triplets: ********** 
    
    template <class Archive, typename _Scalar>
    void save(Archive & ar, const Eigen::Triplet<_Scalar>& m, const unsigned int)
    {
        ar << m.row();
        ar << m.col();
        ar << m.value();
    }
    
    
    template <class Archive, typename _Scalar>
    void load(Archive& ar, Eigen::Triplet<_Scalar>& m, const unsigned int)
    {
        typename Eigen::SparseMatrix<_Scalar>::StorageIndex row, col;
        _Scalar value;
        
        ar >> row;
        ar >> col;
        ar >> value;
        
        m = Eigen::Triplet<_Scalar>(row, col, value);
    }
    
    
    template <class Archive, class _Scalar>
    void serialize(Archive& ar, Eigen::Triplet<_Scalar>& m, const unsigned int version)
    {
        split_free(ar, m, version);
    }
    
    
    // ********** SparseMatrices: ********** 
    
    template <class Archive, typename _Scalar, int _Options, typename _Index>
    void save(Archive& ar, const Eigen::SparseMatrix<_Scalar, _Options, _Index>& m, const unsigned int)
    {
        _Index innerSize = m.innerSize();
        _Index outerSize = m.outerSize();
        
        typedef typename Eigen::Triplet<_Scalar> Triplet;
        std::vector<Triplet> triplets;
        
        for (_Index i=0; i < outerSize; ++i) {
            for (typename Eigen::SparseMatrix<_Scalar, _Options, _Index>::InnerIterator it(m,i); it; ++it) {
                triplets.push_back( Triplet(it.row(), it.col(), it.value()) );
            }
        }
            
        ar << innerSize;
        ar << outerSize;
        ar << triplets;
    }
    
    
    template <class Archive, typename _Scalar, int _Options, typename _Index>
    void load(Archive & ar, Eigen::SparseMatrix<_Scalar, _Options, _Index>& m, const unsigned int)
    {
        _Index innerSize;
        _Index outerSize;
        
        ar >> innerSize;
        ar >> outerSize;
        
        _Index rows = (m.IsRowMajor)? outerSize : innerSize;
        _Index cols = (m.IsRowMajor)? innerSize : outerSize;
        
        m.resize(rows, cols);
        
        typedef typename Eigen::Triplet<_Scalar> Triplet;
        std::vector<Triplet> triplets;
        
        ar >> triplets;
        
        m.setFromTriplets(triplets.begin(), triplets.end());
    }
    
    
    template <class Archive, typename _Scalar, int _Options, typename _Index>
    void serialize(Archive& ar, Eigen::SparseMatrix<_Scalar,_Options,_Index>& m, const unsigned int version)
    {
        split_free(ar, m, version);
    }
    
    
    
}
