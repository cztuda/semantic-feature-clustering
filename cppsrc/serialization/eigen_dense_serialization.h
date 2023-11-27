#pragma once

/**
 * Sources:
 * https://gist.github.com/artivis/288898ff877130e9a58ed04aeb9e59b5
 * https://github.com/artivis/boost_serialization_helper/blob/master/save_load_eigen.h
 * https://gist.github.com/JuantAldea/e0a7f8c80a93fdc01482
 * https://github.com/libigl/libigl/blob/main/include/igl/serialize.h
 */

#include <Eigen/Dense>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost::serialization
{
    
    // ********** Matrices: ********** 
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void save(Archive & ar, const Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int)
    {
        Eigen::DenseIndex rows(m.rows()), cols(m.cols());
        ar & rows;
        ar & cols;
        ar & make_array(m.data(), (size_t)m.size());
    }
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void load(Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int)
    {
        Eigen::DenseIndex rows,cols;
        ar >> rows;
        ar >> cols;
        m.resize(rows,cols);
        ar >> make_array(m.data(), (size_t)m.size());
    }
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void serialize(Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version)
    {
        split_free(ar,m,version);
    }
    
    
    // ********** Arrays:  ********** 
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void save(Archive & ar, const Eigen::Array<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int)
    {
        Eigen::DenseIndex rows(m.rows()), cols(m.cols());
        ar & rows;
        ar & cols;
        ar & make_array(m.data(), (size_t)m.size());
    }
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void load(Archive & ar, Eigen::Array<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int)
    {
        Eigen::DenseIndex rows,cols;
        ar >> rows;
        ar >> cols;
        m.resize(rows,cols);
        ar >> make_array(m.data(), (size_t)m.size());
    }
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void serialize(Archive & ar, Eigen::Array<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version)
    {
        split_free(ar,m,version);
    }
    
    
    // ********** Transforms: ********** 
    
    template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
    inline void serialize(Archive & ar, Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& t, const unsigned int version)
    {
        serialize(ar, t.matrix(), version);
    }
    
    
    // ********** Quaternions: ********** 
    
    template<class Archive, typename _Scalar, int _Options>
    void serialize(Archive & ar, Eigen::Quaternion<_Scalar, _Options>& q, const unsigned int /*version*/)
    {
      ar & q.w();
      ar & q.x();
      ar & q.y();
      ar & q.z();
    }
    
}

