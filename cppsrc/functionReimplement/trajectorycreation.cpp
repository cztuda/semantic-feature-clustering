#include "trajectorycreation.h"

#include "../util/exceptions.h"
#include "TrajectoryReader.h"

#include <vector>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort
#include <iomanip>

#include <Eigen/LU>

using namespace ofc::functionLib;

template<typename T>
std::vector<size_t> sort_indices(T &v) {
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  stable_sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v(i1) < v(i2);});

  return idx;
}

void asVector(const Eigen::VectorXd& src, std::vector<double>& dest){
    dest.resize(src.size());
    for(int i = 0; i < src.size(); ++i){
        dest[i] = src(i);
    }
}


void divideLinearData(std::vector<double> &data, std::vector<double> *times, std::vector<double> *coefficients, int n){
  for(int i = 0; i < data.size(); i++){
    if(i % (n+1) == 0){
      times->push_back(data.at(i));
    } else {
      coefficients->push_back(data.at(i));
    }
  }
}

void mergeLinearData(std::vector<double>& data, const Eigen::VectorXd& time, const Eigen::MatrixXd& coefficients){
    data.reserve(time.size()+coefficients.rows()*coefficients.cols());
    for(int it = 0; it < time.size(); ++it){
        data.push_back(time(it));
        for(int ie = 0; ie < coefficients.cols(); ++ie){
            data.push_back(coefficients(it, ie));
        }
    }
}

void divideCubicData(std::vector<double> &data, std::vector<double> *times, std::vector<double> *coefficients, int n){
  int i;
  for(i = 0; i < data.size()-n-1; i++){
    if( i % (4*n+2) < 2 ){
      times->push_back(data.at(i));
    } else {
      coefficients->push_back(data.at(i));
    }
  }
  times->push_back(data.at(i++));
  while(i < data.size())
    coefficients->push_back(data.at(i++));
}

void mergeCubicData(std::vector<double>& data, int nx, const Eigen::RowVectorXd& time, const Eigen::MatrixXd& coefficients, const Eigen::VectorXd& lastEntry){
    data.reserve(coefficients.rows()*coefficients.cols()+2*(time.size()-1)+1+nx);
    for(int it = 0; it < time.size()-1; ++it){
        data.push_back(time(it));
        data.push_back(time(it+1));
        for(int idim = 0; idim < nx; ++idim){
            data.push_back(coefficients(it*nx+idim, 0));
            data.push_back(coefficients(it*nx+idim, 1));
            data.push_back(coefficients(it*nx+idim, 2));
            data.push_back(coefficients(it*nx+idim, 3));
        }
    }
    data.push_back(time(time.size()-1));
    for(int i = 0; i < nx; ++i){
        data.push_back(lastEntry(i));
    }
}

void createPiecewiseLinearFunctionStreamFromDataStruct(std::stringstream& s, std::shared_ptr<ofc::functionLib::PiecewiseFunctionData>& fdata){
    using namespace std;
    
    // check sizes of given input struct:
    if(fdata->np != fdata->parameter.size()){
        ofc::exception::InconsistentInputError("createPiecewiseLinearFunctionStreamFromDataStruct: Number of given parameters does not match np.");
    }
    if(fdata->n != fdata->names.size()){
        ofc::exception::InconsistentInputError("createPiecewiseLinearFunctionStreamFromDataStruct: Number of given names does not match n.");
    }
    if(fdata->nphases != fdata->ngridpts.size()){
        ofc::exception::InconsistentInputError("createPiecewiseLinearFunctionStreamFromDataStruct: Size of given gridpoints array does not match nphases.");
    }
    int checkn = 0;
    for(auto ngrdpts : fdata->ngridpts){
        checkn += ngrdpts*(fdata->n+1);
    }
    if(fdata->doubles.size() != checkn){
        ofc::exception::InconsistentInputError("createPiecewiseLinearFunctionStreamFromDataStruct: Unexpected size of data array 'doubles'.");
    }
    
    // set header infos
    s << "    " << fdata->n << "    " << fdata->np << "\n    " << fdata->nphases;
    for(int i = 0; i < fdata->nphases; ++i){
        s << "\n    " << fdata->ngridpts[i];
    }
    s << "\n  " << setprecision(16) << fdata->t0 << "    " << setprecision(16) << fdata->tf << "\n";
    
    // set names
    for(int i = 0; i < fdata->n; i++){
        s << fdata->names[i] << "\n";
    }
    
    // set data
    int it = 0;
    for(int phase = 0; phase < fdata->nphases; ++phase){
        for(int grdpt = 0; grdpt < fdata->ngridpts[phase]; ++grdpt){
            s << "  " << setprecision(16) << fdata->doubles[it++];
            for(int dim = 0; dim < fdata->n; ++dim){
                s << "  " << setprecision(16) << fdata->doubles[it++];
            }
            s << "\n";
        }
    }
    
    auto p_it = fdata->parameter.begin();
    it = 0;
    while(p_it != fdata->parameter.end()){
        s << "  " << setprecision(16) << *p_it;
        p_it++; it++;
        if(it == 6){
            s << "\n";
            it = 0;
        }
    }
}

void createPiecewiseCubicFunctionStreamFromDataStruct(std::stringstream& s, const std::shared_ptr<ofc::functionLib::PiecewiseFunctionData>& fdata){
    using namespace std;
    
    // check sizes of given input struct:
    if(fdata->n != fdata->names.size()){
        ofc::exception::InconsistentInputError("createPiecewiseCubicFunctionStreamFromDataStruct: Number of given names does not match n.");
    }
    if(fdata->nphases != fdata->ngridpts.size()){
        ofc::exception::InconsistentInputError("createPiecewiseCubicFunctionStreamFromDataStruct: Size of given gridpoints array does not match nphases.");
    }
    int checkn = 0;
    for(auto ngrdpts : fdata->ngridpts){
        checkn += (ngrdpts-1)*(fdata->n*4+2) + 1 + fdata->n;
    }
    if(fdata->doubles.size() != checkn){
        ofc::exception::InconsistentInputError("createPiecewiseCubicFunctionStreamFromDataStruct: Unexpected size of data array 'doubles'.");
    }
    
    // set header infos
    s << "    " << fdata->n << " " << fdata->estadj << "    0\n";
    s << "    " << fdata->nphases;
    for(int i = 0; i < fdata->nphases; ++i){
        s << "\n    " << fdata->ngridpts[i];
    }
    s << "\n  " << setprecision(16) << fdata->t0 << "    " << setprecision(16) << fdata->tf << "\n";
    
    // set names
    for(int i = 0; i < fdata->n; i++){
        s << fdata->names[i] << "\n";
    }
    
    // set data
    int it = 0;
    for(int phase = 0; phase < fdata->nphases; ++phase){
        for(int grdpt = 0; grdpt < fdata->ngridpts[phase]-1; ++grdpt){
            s << "  " << setprecision(16) << fdata->doubles[it++];
            s << "     " << setprecision(16) << fdata->doubles[it++] << "\n";
            for(int dim = 0; dim < fdata->n; ++dim){
                s << "  " << setprecision(16) << fdata->doubles[it++];
                s << "  " << setprecision(16) << fdata->doubles[it++];
                s << "  " << setprecision(16) << fdata->doubles[it++];
                s << "  " << setprecision(16) << fdata->doubles[it++];
                s << "\n";
            }
        }
        s << "  " << setprecision(16) << fdata->doubles[it++] << "\n";
        for(int dim = 0; dim < fdata->n; ++dim){
            s << setprecision(16) << fdata->doubles[it++] << "\n";
        }
    }
}

void ofc::functionLib::createPiecewiseLinearInterpolation(ofc::functionLib::PiecewiseFunction ** traj, std::shared_ptr<ofc::functionLib::PiecewiseFunctionData>& functionData, const Eigen::VectorXd& t, const Eigen::MatrixXd& pts)
{
    Eigen::VectorXd time = t;
    Eigen::MatrixXd data = (pts.rows() != time.size()) ? pts.transpose() : pts;
    
    if(data.rows() != time.size()){
        ofc::exception::InconsistentInputError("createPiecewiseLinearInterpolation: Number of times and number of data points must be the same.");
    }
    
    const int n = data.rows();
    const int nx = data.cols();
    
    std::vector<size_t> indices = sort_indices(time);
    time = time(indices);
    data = data(indices, Eigen::all);
    
    functionData->n = nx;
    functionData->np = 0;
    functionData->nphases = 1;
    functionData->estadj = 'F';
    functionData->t0 = time(0);
    functionData->tf = time(time.size()-1);
    std::vector<uint> ngrdpts(1);
    ngrdpts[0] = n;
    functionData->ngridpts = ngrdpts;
    for(int i = 0; i < nx; ++i){
        functionData->names.push_back("dim_"+std::to_string(i));
    }
    
    mergeLinearData(functionData->doubles, time, data);
    
    std::stringstream s;
    createPiecewiseLinearFunctionStreamFromDataStruct(s, functionData);
        
    
    ofc::functionLib::TrajectoryReader reader;    
    reader.read(s, traj);    
}


void createPiecewiseCubicInterpolationFromCoefficients(ofc::functionLib::PiecewiseFunction** pwtraj, std::shared_ptr<ofc::functionLib::PiecewiseFunctionData>& fdata, int n, int nx, const Eigen::RowVectorXd& time, const Eigen::MatrixXd& coefficients, const Eigen::VectorXd& lastEntry){
    
    fdata->n = nx;
    fdata->np = 0;
    fdata->nphases = 1;
    fdata->estadj = 'F';
    fdata->t0 = time(0);
    fdata->tf = time(time.size()-1);
    std::vector<uint> ngrdpts(1);
    ngrdpts[0] = n;
    fdata->ngridpts = ngrdpts;
    for(int i = 0; i < nx; ++i){
        fdata->names.push_back("dim_"+std::to_string(i));
    }
    
    mergeCubicData(fdata->doubles, nx, time, coefficients, lastEntry);
    
    std::stringstream s;
    createPiecewiseCubicFunctionStreamFromDataStruct(s, fdata);
    
    ofc::functionLib::TrajectoryReader reader;    
    reader.read(s, pwtraj);
}


template<typename T>
void diff(Eigen::VectorX<T>& d, const Eigen::VectorX<T>& v){
    d.resize(v.size()-1);
    for(int i = 0; i < v.size()-1; ++i){
        d(i) = v(i+1)-v(i);
    }
}
template<typename T>
void diff(Eigen::RowVectorX<T>& d, const Eigen::RowVectorX<T>& v){
    d.resize(v.size()-1);
    for(int i = 0; i < v.size()-1; ++i){
        d(i) = v(i+1)-v(i);
    }
}
template<typename T>
void diffRows(Eigen::MatrixX<T>& d, const Eigen::MatrixX<T>& v){
    d.resize(v.rows()-1, v.cols());
    for(int i = 0; i < v.rows()-1; ++i){
        d(i, Eigen::all) = v(i+1, Eigen::all)-v(i, Eigen::all);
    }
}
template<typename T>
void diffCols(Eigen::MatrixX<T>& d, const Eigen::MatrixX<T>& v){
    d.resize(v.rows(), v.cols()-1);
    for(int i = 0; i < v.rows()-1; ++i){
        d(Eigen::all, i) = v(Eigen::all, i+1)-v(Eigen::all, i);
    }
}


void getMoments(Eigen::MatrixXd& M, const Eigen::RowVectorXd& dt, const Eigen::MatrixXd& dy){
    // natural boundary condition:
    const double mue0 = 1;
    const double muen = 1;
    const double lambda0 = 0;
    const double lambdan = 0;
    Eigen::RowVectorXd b0 = Eigen::RowVectorXd::Zero(dy.rows());
    Eigen::RowVectorXd bn = b0;
    
    Eigen::RowVectorXd hi2 = dt/6;
    
    Eigen::RowVectorXd diag(dt.size()+1);
    diag(0) = mue0;
    diag(Eigen::seq(1, Eigen::last-1)) = dt(Eigen::seq(0, Eigen::last-1)) + dt(Eigen::seq(1, Eigen::last))/3;
    diag(diag.size()-1) = muen;
    
    Eigen::MatrixXd D = diag.asDiagonal();
    
    Eigen::RowVectorXd sdiag(dt.size());
    sdiag(0) = lambda0;
    sdiag(Eigen::seq(1, Eigen::last)) = hi2(Eigen::seq(1, Eigen::last));
    
    D.diagonal(1) = sdiag;
    
    sdiag(Eigen::seq(0, Eigen::last-1)) = hi2(Eigen::seq(0, Eigen::last-1));
    sdiag(sdiag.size()-1) = lambdan;
    
    D.diagonal(-1) = sdiag;
    
    Eigen::MatrixXd b(dt.size()+1, b0.size());
    b(0, Eigen::all) = b0;
    b(Eigen::seq(1, Eigen::last-1), Eigen::all) = (dy(Eigen::all, Eigen::seq(1, Eigen::last)).array().rowwise() / dt(Eigen::seq(1, Eigen::last)).array() - 
    dy(Eigen::all, Eigen::seq(0, Eigen::last-1)).array().rowwise() / dt(Eigen::seq(0, Eigen::last-1)).array()).transpose();
    b(b.rows()-1, Eigen::all) = bn;
    
    M = D.lu().solve(b);
    M.transposeInPlace();
}

void buildCoefficientMatrix(Eigen::MatrixXd& M, const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, const Eigen::MatrixXd& c, const Eigen::MatrixXd& d){
    const int n = a.rows()*a.cols();
    M.resize(n, 4);
    M.col(0) = a.reshaped(n, 1);
    M.col(1) = b.reshaped(n, 1);
    M.col(2) = c.reshaped(n, 1);
    M.col(3) = d.reshaped(n, 1);
}



void ofc::functionLib::createPiecewiseCubicInterpolation(ofc::functionLib::PiecewiseFunction ** traj, std::shared_ptr<ofc::functionLib::PiecewiseFunctionData>& functionData, const Eigen::VectorXd& t, const Eigen::MatrixXd& pts)
{
    Eigen::RowVectorXd time = t;
    Eigen::MatrixXd data = (pts.cols() != time.size()) ? pts.transpose() : pts;
    
    if(data.cols() != time.size()){
        ofc::exception::InconsistentInputError("createPiecewiseCubicInterpolation: Number of times and number of data points must be the same.");
    }
        
    std::vector<size_t> indices = sort_indices(time);
    time = time(indices);
    data = data(Eigen::all, indices);
    
    Eigen::RowVectorXd dt;
    Eigen::MatrixXd dy, M, M1, M2, a, b, c, d;
    diff(dt, time);
    diffCols(dy, data);
    getMoments(M, dt, dy);
    M1 = M(Eigen::all, Eigen::seq(0, Eigen::last-1));
    M2 = M(Eigen::all, Eigen::seq(1, Eigen::last));
    
    a = data(Eigen::all, Eigen::seq(0, Eigen::last-1));
    b = dy.array().rowwise()/dt.array() - ((2*M1+M2)/6).array().rowwise() * dt.array();
    c = M1/2;
    d = (M2-M1).array().rowwise() / (6*dt).array();
    
    b = b.array().rowwise()*dt.array();
    c = c.array().rowwise() * (dt.array().pow(2));
    d = d.array().rowwise() * (dt.array().pow(3));

    Eigen::MatrixXd coefficients;
    buildCoefficientMatrix(coefficients, a, b, c, d);
    
    const int n = data.cols();
    const int nx = data.rows();    
    Eigen::VectorXd lastEntry = data.col(data.cols()-1);
    
    createPiecewiseCubicInterpolationFromCoefficients(traj, functionData, n, nx, time, coefficients, lastEntry);
}

void ofc::functionLib::createPiecewiseCubicInterpolation(ofc::functionLib::PiecewiseFunction ** traj, std::shared_ptr<ofc::functionLib::PiecewiseFunctionData>& functionData, const Eigen::VectorXd& t, const Eigen::MatrixXd& pts, const Eigen::MatrixXd& dpts)
{
    Eigen::RowVectorXd time = t;
    Eigen::MatrixXd data = (pts.cols() != time.size()) ? pts.transpose() : pts;
    Eigen::MatrixXd ddata = (dpts.cols() != time.size()) ? dpts.transpose() : dpts;
    
    if(data.cols() != time.size() || ddata.cols() != time.size()){
        ofc::exception::InconsistentInputError("createPiecewiseCubicInterpolation: Number of times and number of data (or ddata) points must be the same.");
    }
    
    std::vector<size_t> indices = sort_indices(time);
    time = time(indices);
    data = data(Eigen::all, indices);
    ddata = ddata(Eigen::all, indices);
    
    Eigen::RowVectorXd dt;
    Eigen::MatrixXd y1, dy1, dy2, h1, h2, a, b, c, d;
    
    diff(dt, time);
    y1 = data(Eigen::all, Eigen::seq(0, Eigen::last-1));
    dy1 = ddata(Eigen::all, Eigen::seq(0, Eigen::last-1));
    dy2 = ddata(Eigen::all, Eigen::seq(1, Eigen::last));
    diffCols(h1, data);
    h2 = dy2.array().rowwise() * dt.array();
    
    a = y1;
    b = dy1.array().rowwise() * dt.array();
    c = -(3*(-h1) + 2*b + h2);
    d = 2*(-h1) + b + h2;
    
    Eigen::MatrixXd coefficients;
    buildCoefficientMatrix(coefficients, a, b, c, d);
    
    const int n = data.cols();
    const int nx = data.rows();    
    Eigen::VectorXd lastEntry = data.col(data.cols()-1);
    
    createPiecewiseCubicInterpolationFromCoefficients(traj, functionData, n, nx, time, coefficients, lastEntry);
}

