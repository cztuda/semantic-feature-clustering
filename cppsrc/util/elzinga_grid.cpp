#include "elzinga_grid.h"

#include <cmath>
#include <algorithm>

#include "../util/exceptions.h"


inline svr_sym maxElement(const std::vector<svr_sym>& v){
    return (v.size() > 0) ? *std::max_element(v.begin(), v.end()) : (svr_sym)(-1);
}

bool getWeightingMatrix(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y, const Eigen::VectorXd& weights, Eigen::MatrixXd& softMatching){
    int nAlphabet = std::max<svr_num>(maxElement(x), maxElement(y))+1;
    
    bool isDefaultMatrix = false;
    if(softMatching.rows()==0 || softMatching.cols()==0){
        isDefaultMatrix = true;
        int n = std::max<int>(weights.size(), nAlphabet);
        softMatching = Eigen::MatrixXd::Identity(n,n);
    }
    else if(softMatching.rows() < nAlphabet || softMatching.rows() != softMatching.cols()){
        return false;
    }
    
    if(weights.size() == 0){
        if(isDefaultMatrix){
            softMatching.diagonal() = Eigen::VectorXd::Ones(softMatching.diagonal().size());
        }
    }
    else if(weights.size() < nAlphabet || weights.size() != softMatching.rows()){
        return false;
    }
    else {
        softMatching.diagonal() = weights;
    }    
    return true;
}

bool SVRspellOptions::prepareOptions(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y) {
    // run-lengths:
    this->_use_run_length = this->_tx.size() > 0 || this->_ty.size() > 0;
    if(this->_use_run_length){
        if(this->_tx.size() != x.size() || this->_ty.size() != y.size()){
            return false;
        }
    }
    // char-wise-properties ("value"):
    this->_use_values = this->_vx.size() > 0 || this->_vy.size() > 0;
    if(this->_use_values){
        if(this->_vx.size() != x.size() || this->_vy.size() != y.size()){
            return false;
        }
    }
    // character weighting and soft-matching of characters:
    if(!getWeightingMatrix(x, y, this->_charWeights, this->_softMatching)){
        return false;
    }
    // gap-weighting:
    this->_use_mode_specific_gap_weight = this->_gapWeighting_xx != nullptr || this->_gapWeighting_yy != nullptr;
    if(this->_use_mode_specific_gap_weight && (this->_gapWeighting_xx == nullptr || this->_gapWeighting_yy == nullptr || this->_gapWeighting == nullptr)){
        return false;
    }
    // got here, so everything is prepared:
    this->_is_prepared = true;
    return true;
}

SVRspellOptions::SVRspellOptions(const Eigen::MatrixXd& softMatchingMatrix, const Eigen::VectorXd& weights, const lwptr lengthWeighting, const gwptr gapWeighting, const gwptr gapWeighting_xx, const gwptr gapWeighting_yy, const std::vector<double>& tx, const std::vector<double>& ty, const std::vector<double>& vx, const std::vector<double>& vy) :
    _is_prepared(false),
    _use_run_length(false),
    _use_mode_specific_gap_weight(false),
    _use_values(false),
    _mode(SVRspellOptions::MODE::XY),
    _softMatching(softMatchingMatrix), 
    _charWeights(weights),
    _lengthWeighting(lengthWeighting),
    _gapWeighting(gapWeighting),
    _gapWeighting_xx(gapWeighting_xx),
    _gapWeighting_yy(gapWeighting_yy),
    _tx(tx),
    _ty(ty),
    _vx(vx),
    _vy(vy)
{}


const Eigen::MatrixXd& SVRspellOptions::softMatching() const { 
    return this->_softMatching;
};

const Eigen::VectorXd& SVRspellOptions::charWeights() const { 
    return this->_charWeights;
};

lwptr SVRspellOptions::lengthWeighting() const {
    return this->_lengthWeighting;
};

gwptr SVRspellOptions::gapWeighting() const {
    switch(this->_mode){
        case MODE::XX:
            return this->_gapWeighting_xx;
        case MODE::YY:
            return this->_gapWeighting_yy;
        case MODE::XY:
            return this->_gapWeighting;
    }
    return this->_gapWeighting; //default return value
};

const std::vector<svr_val>& SVRspellOptions::tx() const {
    return (_mode == MODE::XX || _mode == MODE::XY) ? this->_tx : this->_ty;
}; 

const std::vector<svr_val>& SVRspellOptions::ty() const {
    return (_mode == MODE::YY || _mode == MODE::XY) ? this->_ty : this->_tx;
};

const std::vector<double> & SVRspellOptions::vx() const {
    return (_mode == MODE::XX || _mode == MODE::XY) ? this->_vx : this->_vy;
}

const std::vector<double> & SVRspellOptions::vy() const {
    return (_mode == MODE::YY || _mode == MODE::XY) ? this->_vy : this->_vx;
}


bool SVRspellOptions::isPrepared() const {
    return _is_prepared;
};

bool SVRspellOptions::useRunLength() const {
    return _use_run_length;
}

SVRspellOptions::MODE SVRspellOptions::mode() const {
    return this->_mode;
}

void SVRspellOptions::mode(const SVRspellOptions::MODE mode) const {
    this->_mode = mode;
}

bool SVRspellOptions::useModeSpecificGapWeight() const {
    return this->_use_mode_specific_gap_weight;
}

bool SVRspellOptions::useValues() const
{
    return this->_use_values;
}



////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


double SVRspell(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y, const SVRspellOptions& options){
    if(options.gapWeighting() == nullptr){
        return grid_algorithm(x, y, options);
    }
    else {
        return trail_algorithm(x, y, options);
    }
}


inline double euclid(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y, const SVRspellOptions& options, double& xy, double& xx, double& yy){
    options.mode(SVRspellOptions::MODE::XX);
    xx = (x.size()>0) ? SVRspell(x, x, options) : 0;
    
    options.mode(SVRspellOptions::MODE::YY);
    yy = (y.size()>0) ? SVRspell(y, y, options) : 0;
    
    options.mode(SVRspellOptions::MODE::XY);
    xy = (x.size()>0 && y.size()>0) ? SVRspell(x, y, options) : 0;
    
    return std::sqrt(xx + yy - 2*xy);
}
    
double distance(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y, SVRspellOptions& options, double& xy, double& xx, double& yy){
    if(!options.isPrepared() && !options.prepareOptions(x, y)){
        throw ofc::exception::InconsistentInputError("The given input is inconsistent.");
    }
    return euclid(x, y, options, xy, xx, yy);
}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


inline double sumMue (double (*lw)(const svr_num&, const double&), 
                      const svr_num& k, 
                      const double& mue_k, 
                      const double& mue) {
    if (lw==nullptr) {
        return mue+mue_k; // default case
    } 
    else {
        return mue+lw(k, mue_k); // apply subsequence length weighting function
    }
}

inline double setMij1(bool useWeights, const std::vector<double>& weights, const std::vector<svr_sym>& x, const int i){
    if(useWeights){
        return weights[x[i]]*weights[x[i]];
    }
    else {
        return 1;
    }
}

inline void summation(Eigen::MatrixXd& M, svr_val& s, svr_val& tmp, const svr_num& i, const svr_num& j){
    tmp = s;
    s += M(i, j);
    M(i, j) = tmp;
}


double grid_algorithm(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y, const SVRspellOptions& options){
    svr_num r = x.size();
    svr_num c = y.size();
    
    auto lengthWeighting = options.lengthWeighting();
    const std::vector<double>& tx = options.tx();
    const std::vector<double>& ty = options.ty();
    const std::vector<double>& vx = options.vx();
    const std::vector<double>& vy = options.vy();
    const Eigen::MatrixXd& softMatching = options.softMatching();
    
    svr_val s_m = 0;
    svr_val s_u, s_v, s_w;
    svr_val t;
    Eigen::MatrixXd M1 = Eigen::MatrixXd::Zero(r, c);
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(r, c);
    Eigen::MatrixXd U1, U, V1, V, W1, W;
    if(options.useRunLength()){
        U1 = Eigen::MatrixXd::Zero(r, c);
        U = Eigen::MatrixXd::Zero(r, c);
        V1 = Eigen::MatrixXd::Zero(r, c);
        V = Eigen::MatrixXd::Zero(r, c);
        W1 = Eigen::MatrixXd::Zero(r, c);
        W = Eigen::MatrixXd::Zero(r, c);
    }
    
    double mue = 0;
    
    for(svr_num i = 0; i < r; ++i){
        for(svr_num j = 0; j < c; ++j){
            if(softMatching(x[i], y[j]) > 0){
                M1(i, j) = softMatching(x[i], y[j]);
                if(options.useValues()){
                    M1(i, j) *= vx[i]*vy[j];
                }
                if(options.useRunLength()){
                    U1(i, j) = tx[i];
                    V1(i, j) = ty[j];
                    W1(i, j) = tx[i]*ty[j];  
                    s_w += W1(i, j);
                }                
                else {
                    s_m += M1(i, j);
                }
            }
        }
    }
    M = M1;
    if(options.useRunLength()){
        U = U1;
        V = V1;
        W = W1;
        mue = s_w;
    }
    else {
        mue = s_m;
    }
    
    if(mue==0){
        return sumMue(lengthWeighting, 0, 1, 0);;
    }
    
    int k = 1;
    while(r > 0 && c > 0){
        for(svr_num i = 0; i < r; ++i){
            s_m = 0;
            s_u = s_v = s_w = 0;
            for(int j = c-1; j >= 0; --j){
                summation(M, s_m, t, i, j);
                if(options.useRunLength()){
                    summation(U, s_u, t, i, j);
                    summation(V, s_v, t, i, j);
                    summation(W, s_w, t, i, j);
                }
            }
        }
        
        double mue_k = 0;
        for(svr_num j = 0; j < c; ++j){
            s_m = 0;
            s_u = s_v = s_w = 0;
            for(int i = r-1; i >= 0; --i){                
                summation(M, s_m, t, i, j);
                if(options.useRunLength()){
                    summation(U, s_u, t, i, j);
                    summation(V, s_v, t, i, j);
                    summation(W, s_w, t, i, j);
                }
            }
        }
        M = M.cwiseProduct(M1);
        if(options.useRunLength()){
            W = M1.cwiseProduct(W + U1.cwiseProduct(V) + V1.cwiseProduct(U) + M.cwiseProduct(W1));
            U = M1.cwiseProduct(U + U1.cwiseProduct(M));
            V = M1.cwiseProduct(V + V1.cwiseProduct(M));
            mue_k = W.sum();
        }
        else {
            mue_k = M.sum();
        }        
        
        if(mue_k == 0){
            return sumMue(lengthWeighting, 0, 1, mue);
        }
        mue = sumMue(lengthWeighting, k, mue_k, mue);
        --r;
        --c;
        ++k;
    }
    return sumMue(lengthWeighting, 0, 1, mue);
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

inline void applyGapWeighting(Eigen::MatrixXd& Q,
                           const std::vector<std::vector<svr_num>>& D, 
                           const svr_num i, 
                           const svr_num j,
                           double (*wf)(const svr_num&, const svr_num&, const svr_num&, const svr_num&),
                           const std::vector<svr_num>& I1,
                           const std::vector<svr_num>& I2){
    if(wf != nullptr){
        for(svr_num m = 0; m < D[i].size(); ++m){
            Q(i, j) += Q(D[i][m], j-1) * wf(I1[i], I2[i], I1[D[i][m]], I2[D[i][m]]);
        }
    }
    else {
        Q(i, j) = Q(D[i], j-1).sum(); // replaces lines 25-27
    }
//     Q(i, j) *= Q(i, 0); // soft-matching, if no soft-matching, then Q(i,1)==1
}


double trail_algorithm(const std::vector<svr_sym>& x, 
                      const std::vector<svr_sym>& y, 
                      const SVRspellOptions& options){
    const svr_num r = x.size();
    const svr_num c = y.size();
    
    auto gapWeighting = options.gapWeighting();
    auto lengthWeighting = options.lengthWeighting();
    const std::vector<double>& tx = options.tx();
    const std::vector<double>& ty = options.ty();
    const std::vector<double>& vx = options.vx();
    const std::vector<double>& vy = options.vy();
    const Eigen::MatrixXd& softMatchingMatrix = options.softMatching();
    
    const int M = std::min(r, c);
    std::vector<svr_num> I1, I2;
//     I1.reserve(M); I2.reserve(M);
    
    // construct trail (I1, I2):
    for(svr_num i = 0; i < r; ++i){
        for(svr_num j = 0; j < c; ++j){
            if(softMatchingMatrix(x[i], y[j]) > 0){
                I1.push_back(i); // in paper: "push_back(k-1);", but this does not make sense!
                I2.push_back(j);
            }
        }
    }
    const int k = I1.size();
    std::vector<std::vector<svr_num>> D(k);
    
    // construct array of sets D:
    for(svr_num i = 0; i < k-1; ++i){
        for(svr_num j = i+1; j < k; ++j){
            if(I1[i] < I1[j] && I2[i] < I2[j]){
                D[i].push_back(j);
            }
        }
    }
    
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(k, M);
    Eigen::MatrixXd U, V, W;
    if(options.useRunLength()){
        U.setZero(k, M);
        V.setZero(k, M);
        W.setZero(k, M);
    }
    for(svr_num i = 0; i < k; ++i){
        Q(i, 0) = softMatchingMatrix(x[I1[i]], y[I2[i]]);
        if(options.useValues()){
            Q(i, 0) *= vx[I1[i]]*vy[I2[i]];
        }
        if(options.useRunLength()){
            U(i, 0) = tx[I1[i]];
            V(i, 0) = ty[I2[i]];
            W(i, 0) = tx[I1[i]]*ty[I2[i]];
        }
    }
//     Q.col(0) = Eigen::VectorXd::Ones(k); // replaces lines 18-20
    
    double mue = 0;
    if(options.useRunLength()){
        mue = sumMue(lengthWeighting, 1, W.col(0).sum(), 0);
    } else {
        mue = sumMue(lengthWeighting, 1, Q.col(0).sum(), 0);
    } 
    double mue_j = 0;
    for(svr_num j = 1; j < M; ++j){
        mue_j = 0;
        for(svr_num i = 0; i < k; ++i){
            applyGapWeighting(Q, D, i, j, gapWeighting, I1, I2); // lines 25-27
            if(options.useRunLength()){
                U(i, j) = U(D[i], j-1).sum();
                V(i, j) = V(D[i], j-1).sum();
                W(i, j) = W(D[i], j-1).sum();
            }
        }
        Q.col(j) = Q.col(j).cwiseProduct(Q.col(0));
        if(options.useRunLength()){
            W.col(j) = Q.col(0).cwiseProduct(W.col(j) + U.col(0).cwiseProduct(V.col(j)) + V.col(0).cwiseProduct(U.col(j)) + Q.col(j).cwiseProduct(W.col(0)));
            U.col(j) = Q.col(0).cwiseProduct(U.col(j) + U.col(0).cwiseProduct(Q.col(j)));
            V.col(j) = Q.col(0).cwiseProduct(V.col(j) + V.col(0).cwiseProduct(Q.col(j)));
            mue_j = W.col(j).sum();
        }
        else{
            mue_j = Q.col(j).sum();
        }
        
        if(mue_j == 0){
            return sumMue(lengthWeighting, 0, 1, mue);
        }
        mue  = sumMue(lengthWeighting, j+1, mue_j, mue);
    }
    return sumMue(lengthWeighting, 0, 1, mue);
}


